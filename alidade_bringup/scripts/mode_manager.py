#!/usr/bin/env python3
"""Gestionnaire de modes : un seul noeud decide de ce qui tourne.

Services (alidade_msgs) :
    /mode_manager/set_mode    changer de mode, avec la carte pour LOCALIZATION
    /mode_manager/save_map    sauvegarder la carte en cours (MAPPING)
    /mode_manager/list_maps   cartes disponibles
Publie /robot/state (RobotState, 2 Hz, transient_local).

Il pilote par cycle de vie slam_toolbox, map_server, amcl et map_saver (tous
lances 'unconfigured' par les launch), et nav2 a travers son lifecycle
manager (STARTUP / PAUSE). La logique des transitions est dans
alidade_bringup/modes.py, testee sans ROS ; ici on execute les etapes, on
observe (TF, AMCL, nav2, diagnostics) et on publie.

Reconciliation : nav2 tourne si et seulement si le mode navigue ET map -> odom
existe (slam_toolbox des le premier scan, amcl apres la pose initiale). Sans
cette regle, la costmap globale expire a l'activation et nav2 abandonne.
"""
from __future__ import annotations

import math
import os
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap, ManageLifecycleNodes, SaveMap
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from slam_toolbox.srv import SerializePoseGraph
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

try:  # paquet tiers (system/sources.sh) : sans lui, EXPLORATION est refuse proprement
    from frontier_exploration_ros2.srv import ControlExploration
except ImportError:
    ControlExploration = None

from alidade_bringup import modes
from alidade_msgs.msg import RobotState
from alidade_msgs.srv import ListMaps, SaveMap as SaveMapSrv, SetMode

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
ACTIVE_GOAL = {GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING}
GOAL_TEXT = {GoalStatus.STATUS_ACCEPTED: 'accepted', GoalStatus.STATUS_EXECUTING: 'executing',
             GoalStatus.STATUS_CANCELING: 'canceling', GoalStatus.STATUS_SUCCEEDED: 'succeeded',
             GoalStatus.STATUS_CANCELED: 'canceled', GoalStatus.STATUS_ABORTED: 'aborted'}


class Step(Exception):
    """Une etape a echoue ; le message est destine a l'operateur."""


class ModeManager(Node):
    def __init__(self):
        super().__init__('mode_manager')
        self.declare_parameter('initial_mode', 'IDLE')
        self.declare_parameter('maps_dir', '~/alidade_maps')
        self.declare_parameter('lifecycle_timeout', 30.0)
        self.declare_parameter('nav_timeout', 120.0)
        self.declare_parameter('map_fresh_s', 5.0)
        self.maps_dir = Path(os.path.expanduser(self.get_parameter('maps_dir').value))
        self.maps_dir.mkdir(parents=True, exist_ok=True)
        self.lc_timeout = float(self.get_parameter('lifecycle_timeout').value)
        self.nav_timeout = float(self.get_parameter('nav_timeout').value)
        self.map_fresh_s = float(self.get_parameter('map_fresh_s').value)

        # Etat observe.
        self.lock = threading.Lock()          # protege les champs ci-dessous
        self.busy = threading.Lock()          # une seule transition a la fois
        self.mode = modes.IDLE
        self.substate = 'demarrage'
        self.transitioning = False
        self.current_map = ''
        self.slam_active = False       # etats etablis par nos propres transitions
        self.amcl_active = False
        self.last_map = 0.0            # slam_toolbox publie sa carte, donc map -> odom
        self.amcl_converged = False    # amcl a publie une pose depuis son activation
        self.amcl_sigma = None
        self.nav_active = False
        self.nav_busy = False
        self.nav_started_once = False
        self.goal_status = ''
        self.goal_active = False
        self.distance_remaining = 0.0
        self.health = DiagnosticStatus.STALE
        self.errors: list[str] = []
        self.sensors = {'lidar': False, 'camera': False, 'imu': False, 'pico': False}

        io = ReentrantCallbackGroup()
        self.io = io
        # Pas d'abonnement a /tf : 118 Hz de transformations desserialisees en
        # Python pour une seule information (le repere map existe-t-il ?), soit
        # le poste de depense principal du noeud. Chaque source de localisation
        # a un signal propre bien moins couteux, et qui dit la meme chose.
        self.create_subscription(OccupancyGrid, '/map', self.on_map, LATCHED, callback_group=io)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 10, callback_group=io)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.on_goal_status,
                                 LATCHED, callback_group=io)
        self.create_subscription(NavigateToPose.Impl.FeedbackMessage, '/navigate_to_pose/_action/feedback',
                                 self.on_goal_feedback, 10, callback_group=io)
        self.create_subscription(DiagnosticArray, '/diagnostics_agg', self.on_diagnostics, 10, callback_group=io)

        self.lc = {name: (self.create_client(GetState, f'/{name}/get_state', callback_group=io),
                          self.create_client(ChangeState, f'/{name}/change_state', callback_group=io))
                   for name in ('slam_toolbox', 'amcl', 'map_server', 'map_saver')}
        self.cli_load_map = self.create_client(LoadMap, '/map_server/load_map', callback_group=io)
        self.cli_map_params = self.create_client(SetParameters, '/map_server/set_parameters', callback_group=io)
        self.cli_save_map = self.create_client(SaveMap, '/map_saver/save_map', callback_group=io)
        self.cli_serialize = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map', callback_group=io)
        self.cli_nav_manage = self.create_client(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes',
                                                 callback_group=io)
        self.cli_nav_active = self.create_client(Trigger, '/lifecycle_manager_navigation/is_active', callback_group=io)
        self.cli_cancel = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal', callback_group=io)
        self.cli_explore = None
        if ControlExploration is not None:
            self.cli_explore = self.create_client(ControlExploration, '/control_exploration', callback_group=io)
        self.explore_started_at = 0.0
        self.explore_done = False
        self.create_subscription(Empty, '/exploration_complete', self.on_explore_done, LATCHED, callback_group=io)

        srv = MutuallyExclusiveCallbackGroup()
        self.create_service(SetMode, '/mode_manager/set_mode', self.on_set_mode, callback_group=srv)
        self.create_service(SaveMapSrv, '/mode_manager/save_map', self.on_save_map, callback_group=srv)
        self.create_service(ListMaps, '/mode_manager/list_maps', self.on_list_maps, callback_group=io)

        self.pub = self.create_publisher(RobotState, '/robot/state', LATCHED)
        self.create_timer(0.5, self.on_tick, callback_group=ReentrantCallbackGroup())
        self.nav_poll_at = 0.0

        threading.Thread(target=self.startup, daemon=True).start()

    # ------------------------------------------------------------------ observation
    def on_map(self, _msg):
        with self.lock:
            self.last_map = time.monotonic()

    def on_amcl(self, msg):
        c = msg.pose.covariance
        with self.lock:
            self.amcl_sigma = math.sqrt(max(c[0], 0.0) + max(c[7], 0.0))
            self.amcl_converged = True

    def on_goal_status(self, msg):
        if not msg.status_list:
            return
        last = msg.status_list[-1]
        with self.lock:
            self.goal_active = any(s.status in ACTIVE_GOAL for s in msg.status_list)
            self.goal_status = GOAL_TEXT.get(last.status, '')
            if not self.goal_active:
                self.distance_remaining = 0.0

    def on_goal_feedback(self, msg):
        with self.lock:
            self.distance_remaining = float(msg.feedback.distance_remaining)

    def on_diagnostics(self, msg):
        levels = {s.name: s for s in msg.status}
        root = levels.get('/Alidade')
        groups = [s for n, s in levels.items() if n.count('/') == 2 and n.startswith('/Alidade/')]

        def ok(name):
            s = levels.get(name)
            return s is not None and s.level == DiagnosticStatus.OK

        def sensor_ok(keyword):
            """Cherche par mot-cle sous Capteurs plutot que par nom exact : ces
            noms sont derives des topics surveilles (telemetry.yaml) et bougent
            avec eux. Sain = au moins un diagnostic trouve, et aucun en defaut."""
            found = [s for n, s in levels.items()
                     if n.startswith('/Alidade/Capteurs/') and keyword in n]
            return bool(found) and all(s.level == DiagnosticStatus.OK for s in found)

        with self.lock:
            self.health = root.level if root else DiagnosticStatus.STALE
            self.errors = [f"{s.name.rsplit('/', 1)[1]} : {s.message}" for s in groups
                           if s.level in (DiagnosticStatus.WARN, DiagnosticStatus.ERROR)]
            self.sensors = {
                'lidar': sensor_ok('scan'),
                'camera': sensor_ok('camera'),
                'imu': sensor_ok('imu'),
                'pico': ok('/Alidade/Pico'),
            }

    def localized(self) -> bool:
        """Le repere map existe et la planification peut tourner.

        En cartographie, slam_toolbox republie /map a map_update_interval tant
        qu'il traite des scans, et publie map -> odom en meme temps : une carte
        fraiche suffit. Avec amcl, en revanche, /amcl_pose n'est publie qu'aux
        mises a jour du filtre -- rien a l'arret -- alors que la transformation,
        elle, continue : c'est donc un drapeau depuis la premiere pose, remis a
        zero a chaque activation.
        """
        # Selon la source ACTIVE, pas selon le mode : TELEOP garde celle qui
        # tournait, et l'etat publie doit dire la verite pendant ce temps.
        with self.lock:
            if self.slam_active:
                return time.monotonic() - self.last_map < self.map_fresh_s
            if self.amcl_active:
                return self.amcl_converged
            return False

    # ------------------------------------------------------------------ appels bloquants
    def call(self, client, request, timeout: float, what: str):
        if not client.wait_for_service(timeout_sec=min(timeout, 10.0)):
            raise Step(f'{what} : service {client.srv_name} absent')
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise Step(f'{what} : pas de reponse en {timeout:.0f} s')
            time.sleep(0.05)
        return future.result()

    def lc_state(self, name: str) -> int:
        get, _ = self.lc[name]
        return self.call(get, GetState.Request(), self.lc_timeout, f'{name} get_state').current_state.id

    def lc_change(self, name: str, transition: int, label: str):
        _, change = self.lc[name]
        self.set_substate(f'{name} : {label}')
        res = self.call(change, ChangeState.Request(transition=Transition(id=transition)),
                        self.lc_timeout, f'{name} {label}')
        if not res.success:
            raise Step(f'{name} : {label} refuse')

    def lc_bring_to_active(self, name: str):
        state = self.lc_state(name)
        if state == State.PRIMARY_STATE_UNCONFIGURED:
            self.lc_change(name, Transition.TRANSITION_CONFIGURE, 'configure')
            state = State.PRIMARY_STATE_INACTIVE
        if state == State.PRIMARY_STATE_INACTIVE:
            self.lc_change(name, Transition.TRANSITION_ACTIVATE, 'activate')

    def lc_deactivate(self, name: str):
        if self.lc_state(name) == State.PRIMARY_STATE_ACTIVE:
            self.lc_change(name, Transition.TRANSITION_DEACTIVATE, 'deactivate')

    def lc_fresh(self, name: str):
        """Repart d'un noeud vierge : cleanup si besoin, puis configure et activate."""
        state = self.lc_state(name)
        if state == State.PRIMARY_STATE_ACTIVE:
            self.lc_change(name, Transition.TRANSITION_DEACTIVATE, 'deactivate')
            state = State.PRIMARY_STATE_INACTIVE
        if state == State.PRIMARY_STATE_INACTIVE:
            self.lc_change(name, Transition.TRANSITION_CLEANUP, 'cleanup')
        self.lc_change(name, Transition.TRANSITION_CONFIGURE, 'configure')
        self.lc_change(name, Transition.TRANSITION_ACTIVATE, 'activate')

    # ------------------------------------------------------------------ etapes
    def step_nav_cancel(self):
        if self.cli_cancel.wait_for_service(timeout_sec=1.0):
            self.set_substate('annulation du but')
            self.call(self.cli_cancel, CancelGoal.Request(), 5.0, 'annulation')

    def step_explore(self, start: bool):
        if self.cli_explore is None:
            raise Step('explorateur absent (frontier_exploration_ros2 non installe)')
        action = ControlExploration.Request.ACTION_START if start else ControlExploration.Request.ACTION_STOP
        self.set_substate('exploration : ' + ('demarrage' if start else 'arret'))
        res = self.call(self.cli_explore,
                        ControlExploration.Request(action=action, delay_seconds=0.0, quit_after_stop=False),
                        15.0, 'control_exploration')
        if not res.accepted:
            raise Step(f'explorateur : {res.message}')
        with self.lock:
            self.explore_started_at = time.monotonic() if start else 0.0
            self.explore_done = False

    def on_explore_done(self, _msg):
        """Plus de frontiere : l'explorateur rentre au depart de lui-meme ; on
        sauvegarde la carte, puis on repasse en MAPPING une fois le retour fini.
        Le topic est transient_local : un vieux message rejoue au demarrage,
        d'ou la garde sur le mode et l'age de l'exploration."""
        with self.lock:
            fresh = self.mode == modes.EXPLORATION and not self.explore_done \
                and time.monotonic() - self.explore_started_at > 5.0
            if fresh:
                self.explore_done = True
        if not fresh:
            return
        self.get_logger().info('exploration terminee : sauvegarde de la carte, retour au depart')
        threading.Thread(target=self.finish_exploration, daemon=True).start()

    def finish_exploration(self):
        name = datetime.now().strftime('exploration_%Y%m%d_%H%M%S')
        self.save_map(name, self.maps_dir / name)
        deadline = time.monotonic() + 180.0
        time.sleep(3.0)
        while time.monotonic() < deadline:
            with self.lock:
                if self.mode != modes.EXPLORATION:
                    return
                if not self.goal_active:
                    break
                self.substate = f'carte {name} sauvegardee, retour au depart'
            time.sleep(1.0)
        ok, msg = self.request_mode(modes.MAPPING)
        self.get_logger().info(f'fin d exploration -> MAPPING : {msg}')

    def step_map_load(self, name: str):
        path = self.maps_dir / name / 'map.yaml'
        if not path.exists():
            raise Step(f"carte '{name}' : {path} introuvable")
        state = self.lc_state('map_server')
        if state == State.PRIMARY_STATE_ACTIVE:
            self.set_substate(f'map_server : charge {name}')
            res = self.call(self.cli_load_map, LoadMap.Request(map_url=str(path)), self.lc_timeout, 'load_map')
            if res.result != LoadMap.Response.RESULT_SUCCESS:
                raise Step(f'map_server : load_map a echoue (code {res.result})')
        else:
            if state == State.PRIMARY_STATE_INACTIVE:
                self.lc_change('map_server', Transition.TRANSITION_CLEANUP, 'cleanup')
            self.set_substate(f'map_server : configure sur {name}')
            param = Parameter(name='yaml_filename',
                              value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=str(path)))
            res = self.call(self.cli_map_params, SetParameters.Request(parameters=[param]), self.lc_timeout,
                            'map_server set_parameters')
            if not res.results or not res.results[0].successful:
                raise Step('map_server : yaml_filename refuse')
            self.lc_bring_to_active('map_server')
        with self.lock:
            self.current_map = name

    def execute(self, plan: modes.Plan):
        for target, action, arg in plan.steps:
            if target == 'nav' and action == 'cancel':
                self.step_nav_cancel()
            elif target == 'slam' and action == 'fresh':
                with self.lock:
                    self.last_map = 0.0   # la carte de la session precedente ne prouve rien
                self.lc_fresh('slam_toolbox')
                self.set_flag('slam_active', True)
            elif target == 'slam' and action == 'deactivate':
                self.lc_deactivate('slam_toolbox')
                self.set_flag('slam_active', False)
            elif target == 'amcl' and action == 'activate':
                # Redemarrage complet, pas une simple reactivation : amcl garde
                # son filtre d'une activation a l'autre et republie aussitot la
                # pose de la session precedente. Le gestionnaire le croyait
                # localise et lancait la planification sur une pose perimee,
                # eventuellement issue d'une AUTRE carte.
                self.set_flag('amcl_converged', False)
                self.lc_fresh('amcl')
                self.set_flag('amcl_active', True)
            elif target == 'amcl' and action == 'deactivate':
                self.lc_deactivate('amcl')
                self.set_flag('amcl_active', False)
                self.set_flag('amcl_converged', False)
            elif target == 'map_server' and action == 'load':
                self.step_map_load(arg)
            elif target == 'explore':
                self.step_explore(action == 'start')
            else:
                raise Step(f'etape inconnue : {target} {action}')

    def set_flag(self, name: str, value: bool):
        with self.lock:
            setattr(self, name, value)

    def snapshot(self) -> modes.Context:
        # (pas 'context' : c'est un attribut de rclpy.Node). Les etats des
        # sources sont ceux que le gestionnaire a lui-meme etablis : pas
        # d'appel de service ici, la reponse a set_mode doit etre immediate.
        with self.lock:
            return modes.Context(current=self.mode, maps=self.list_maps(), current_map=self.current_map,
                                 slam_active=self.slam_active, amcl_active=self.amcl_active)

    def request_mode(self, target: int, map_name: str = '') -> tuple[bool, str]:
        """Verifie et lance la transition ; le resultat se lit dans /robot/state.

        rosbridge coupe un appel de service a 5 s et une transition peut en
        prendre soixante (demarrage de nav2) : on repond des que le plan est
        accepte, et un fil l'execute.
        """
        if not self.busy.acquire(blocking=False):
            return False, 'une transition est deja en cours'
        try:
            plan = modes.plan(self.snapshot(), target, map_name)
        except modes.Refused as why:
            self.busy.release()
            return False, str(why)
        if not plan.steps:
            self.busy.release()
            return True, f'deja en {modes.name_of(target)}'
        with self.lock:
            self.transitioning = True
            self.substate = f'vers {modes.name_of(target)}'
        threading.Thread(target=self.run_plan, args=(plan,), daemon=True).start()
        return True, f'transition vers {modes.name_of(target)} lancee'

    def change_mode(self, target: int, map_name: str = '') -> tuple[bool, str]:
        """Variante bloquante, pour le demarrage."""
        ok, msg = self.request_mode(target, map_name)
        while self.busy.locked():
            time.sleep(0.1)
        with self.lock:
            done = self.mode == target
        return (ok and done), (msg if ok and done else (self.substate or msg))

    def run_plan(self, plan: modes.Plan):
        """Execute un plan accepte ; self.busy est deja pris par request_mode."""
        target = plan.target
        try:
            self.get_logger().info(f'{modes.name_of(self.mode)} -> {modes.name_of(target)} : {plan.summary}')
            try:
                self.execute(plan)
            except Step as why:
                self.get_logger().error(f'transition vers {modes.name_of(target)} : {why}')
                with self.lock:
                    slam_ok = self.slam_active
                if target == modes.EXPLORATION and slam_ok:
                    # L'explorateur manque ou refuse : la carte en cours reste
                    # bonne, on reste en cartographie plutot que de tout couper.
                    with self.lock:
                        self.mode = modes.MAPPING
                        self.transitioning = False
                        self.substate = f'exploration impossible : {why}'
                else:
                    self.fall_back(str(why))
                return
            # Hors verrou : localized() le prend aussi.
            localized = self.localized()
            with self.lock:
                self.mode = target
                self.transitioning = False
                self.substate = 'en attente d une pose initiale' \
                    if target in modes.AMCL_MODES and not localized else ''
                if target in modes.AMCL_MODES:
                    self.current_map = plan.map_name
            self.get_logger().info(f'mode {modes.name_of(target)}')
        finally:
            self.busy.release()

    def fall_back(self, why: str):
        """Apres un echec : plus aucune source de localisation, mode IDLE."""
        for name, flag in (('slam_toolbox', 'slam_active'), ('amcl', 'amcl_active')):
            try:
                self.lc_deactivate(name)
            except Step:
                pass
            self.set_flag(flag, False)
        self.set_flag('amcl_converged', False)
        with self.lock:
            self.mode = modes.IDLE
            self.transitioning = False
            self.substate = f'echec : {why}'

    def set_substate(self, text: str):
        with self.lock:
            self.substate = text

    # ------------------------------------------------------------------ nav2
    def reconcile_nav(self):
        with self.lock:
            mode, busy, active = self.mode, self.nav_busy, self.nav_active
        wanted = modes.nav_wanted(mode, self.localized())
        if busy or wanted == active:
            return
        with self.lock:
            self.nav_busy = True
        threading.Thread(target=self.nav_command, args=(wanted,), daemon=True).start()

    def nav_command(self, start: bool):
        # Premier demarrage : STARTUP (configure + activate). Apres une pause,
        # les noeuds sont 'inactive' et STARTUP tenterait de les reconfigurer :
        # RESUME les reactive seulement.
        if start:
            cmd = ManageLifecycleNodes.Request.RESUME if self.nav_started_once \
                else ManageLifecycleNodes.Request.STARTUP
        else:
            cmd = ManageLifecycleNodes.Request.PAUSE
        label = 'demarrage de nav2' if start else 'pause de nav2'
        try:
            self.set_substate(label)
            res = self.call(self.cli_nav_manage, ManageLifecycleNodes.Request(command=cmd),
                            self.nav_timeout, label)
            with self.lock:
                self.nav_active = bool(res.success) if start else (False if res.success else self.nav_active)
                self.substate = '' if res.success else f'{label} : echec'
            if res.success and start:
                self.nav_started_once = True
            if not res.success:
                self.get_logger().error(f'{label} : le lifecycle manager a refuse')
        except Step as why:
            self.get_logger().warn(str(why))
            with self.lock:
                self.substate = str(why)
        finally:
            with self.lock:
                self.nav_busy = False

    def poll_nav_active(self):
        if not self.cli_nav_active.service_is_ready():
            return
        future = self.cli_nav_active.call_async(Trigger.Request())

        def done(f):
            try:
                with self.lock:
                    self.nav_active = bool(f.result().success)
            except Exception:
                pass
        future.add_done_callback(done)

    # ------------------------------------------------------------------ cartes
    def list_maps(self) -> list[str]:
        return sorted(p.parent.name for p in self.maps_dir.glob('*/map.yaml'))

    def on_list_maps(self, request, response):
        response.names = self.list_maps()
        with self.lock:
            response.current = self.current_map
        return response

    def on_save_map(self, request, response):
        """Lance la sauvegarde et repond tout de suite (rosbridge coupe a 5 s) ;
        le resultat s'affiche dans le sous-etat de /robot/state."""
        with self.lock:
            mode, busy = self.mode, self.transitioning
        if mode not in modes.SLAM_MODES:
            response.ok, response.message = False, 'pas de cartographie en cours'
            return response
        if busy:
            response.ok, response.message = False, 'une transition est en cours'
            return response
        name = request.name.strip() or datetime.now().strftime('carte_%Y%m%d_%H%M%S')
        if '/' in name or name.startswith('.'):
            response.ok, response.message = False, f"nom invalide : '{name}'"
            return response
        folder = self.maps_dir / name
        threading.Thread(target=self.save_map, args=(name, folder), daemon=True).start()
        response.ok, response.message, response.path = True, f'sauvegarde de {name} lancee', str(folder)
        return response

    def save_map(self, name: str, folder: Path):
        folder.mkdir(parents=True, exist_ok=True)
        self.set_substate(f'sauvegarde de {name}')
        try:
            self.lc_bring_to_active('map_saver')
            req = SaveMap.Request(map_topic='map', map_url=str(folder / 'map'), image_format='pgm',
                                  map_mode='trinary', free_thresh=0.25, occupied_thresh=0.65)
            res = self.call(self.cli_save_map, req, 20.0, 'save_map')
            if not res.result:
                raise Step('map_saver a refuse (pas de carte recue ?)')
            res = self.call(self.cli_serialize, SerializePoseGraph.Request(filename=str(folder / 'posegraph')),
                            20.0, 'serialize_map')
            if res.result != SerializePoseGraph.Response.RESULT_SUCCESS:
                self.get_logger().warn('posegraph non sauvegarde (la carte pgm l est)')
        except Step as why:
            self.get_logger().error(f'sauvegarde de {name} : {why}')
            self.set_substate(f'sauvegarde de {name} : {why}')
            return
        with self.lock:
            self.current_map = name
            self.substate = f'carte {name} sauvegardee'
        self.get_logger().info(f'carte {name} sauvegardee dans {folder}')

    # ------------------------------------------------------------------ services et cycle
    def on_set_mode(self, request, response):
        response.ok, response.message = self.request_mode(int(request.mode), request.map_name)
        return response

    def startup(self):
        # map_saver toujours pret ; puis le mode initial demande.
        time.sleep(2.0)
        try:
            self.lc_bring_to_active('map_saver')
        except Step as why:
            self.get_logger().warn(f'map_saver : {why}')
        self.set_substate('')
        initial = self.get_parameter('initial_mode').value.upper()
        target = modes.BY_NAME.get(initial)
        if target is None:
            self.get_logger().error(f"initial_mode '{initial}' inconnu, IDLE")
            target = modes.IDLE
        ok, msg = self.change_mode(target)
        self.get_logger().info(f'mode initial {modes.name_of(target)} : {msg}')
        if not ok:
            with self.lock:
                self.substate = msg
        with self.lock:
            if self.substate == 'demarrage':
                self.substate = ''

    def on_tick(self):
        now = time.monotonic()
        if now > self.nav_poll_at:
            self.nav_poll_at = now + 2.0
            self.poll_nav_active()
        if not self.busy.locked():
            self.reconcile_nav()
        self.publish_state()

    def publish_state(self):
        localized = self.localized()
        msg = RobotState()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            msg.mode = self.mode
            msg.mode_name = modes.name_of(self.mode)
            msg.substate = self.substate
            msg.transitioning = self.transitioning
            if self.slam_active:
                msg.localization_source = 'slam_toolbox'
                msg.localization_quality = 1.0 if localized else 0.0
            elif self.amcl_active:
                msg.localization_source = 'amcl'
                # Ecart-type de position combine, ramene sur un metre : une
                # pose initiale posee a la main vaut deja 0.7 m d'incertitude,
                # et une echelle a 0.5 m affichait 0 % sur une localisation
                # pourtant valide. Au-dela d'un metre dans une piece de six, on
                # est bel et bien perdu.
                sigma = self.amcl_sigma
                msg.localization_quality = max(0.0, min(1.0, 1.0 - sigma)) if sigma is not None and localized else 0.0
            msg.localized = localized
            msg.map_name = self.current_map
            msg.nav_active = self.nav_active
            msg.goal_active = self.goal_active
            msg.goal_status = self.goal_status
            msg.distance_remaining = self.distance_remaining
            # DiagnosticStatus.level est un 'byte' : bytes en Python, pas un int.
            msg.health = self.health[0] if isinstance(self.health, (bytes, bytearray)) else int(self.health)
            msg.errors = list(self.errors)
            msg.lidar_ok = self.sensors['lidar']
            msg.camera_ok = self.sensors['camera']
            msg.imu_ok = self.sensors['imu']
            msg.pico_ok = self.sensors['pico']
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ModeManager()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
