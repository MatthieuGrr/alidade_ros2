"""Odometrie a cap gyroscopique, sans ROS.

La distance vient des roues, le CAP vient du gyromètre. C'est la solution
classique sur une base differentielle, et il n'y a rien d'autre a fusionner
ici : pas de GPS, et l'accelerometre du BNO055 n'est pas calibre (decalage a
zero g de 0.8 m/s2, gravite lue 2 % trop faible), donc inutilisable.

Pourquoi le gyromètre plutot que les roues pour le cap, mesure a l'appui :
biais 0.005 deg/s, soit 0.3 deg de derive par minute, et 0.1 deg/s de bruit ;
les roues, elles, se trompent de 1 a 2 % sur chaque rotation et ne voient pas
du tout un patinage. En revanche le gyromètre ne dit rien de la distance
parcourue : les deux sources sont complementaires, pas concurrentes.

Repli : sans echantillon gyro valide (IMU muette, ou trame de zeros -- le
firmware du Pico envoie des zeros quand la lecture I2C echoue, et `accel z`
nul est impossible sous gravite), on reprend le cap des roues pour ce pas, et
on le signale. Mieux vaut une odometrie degradee qu'un robot qui se croit
immobile en pleine rotation.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

WHEELS = 'roues'
GYRO = 'gyro'


@dataclass
class Gains:
    scale: float = 1.0            # correction d'echelle du gyromètre, mesuree en tours complets
    still_v: float = 0.005        # m/s, en dessous le robot est considere a l'arret...
    still_w: float = 0.02         # rad/s, ... pour reestimer le biais
    still_gyro: float = math.radians(2.0)  # rad/s, et le gyromètre doit etre d'accord :
    #                                        sinon un robot pousse a la main, ou des roues
    #                                        qui patinent, feraient passer une VRAIE rotation
    #                                        pour un biais.
    bias_tau: float = 2.0         # s, constante de temps de l'estimation du biais
    bias_max: float = math.radians(1.0)   # rad/s, garde-fou : le BNO055 compense deja le sien


@dataclass
class Odometry:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    bias: float = 0.0             # rad/s, biais estime du gyromètre
    source: str = WHEELS          # source du cap au dernier pas
    fallbacks: int = 0            # pas ou le gyromètre a manque
    gains: Gains = field(default_factory=Gains)

    def step(self, dt: float, v: float, w_wheel: float, w_gyro: float | None) -> None:
        """Avance l'odometrie de `dt` : `v` et `w_wheel` viennent des roues,
        `w_gyro` du gyromètre (None s'il n'est pas exploitable)."""
        if dt <= 0.0:
            return
        g = self.gains
        if w_gyro is None:
            w = w_wheel
            self.source = WHEELS
            self.fallbacks += 1
        else:
            if abs(v) < g.still_v and abs(w_wheel) < g.still_w and abs(w_gyro) < g.still_gyro:
                # A l'arret, tout ce que lit le gyromètre est son biais.
                k = min(1.0, dt / g.bias_tau)
                self.bias += k * (w_gyro - self.bias)
                self.bias = max(-g.bias_max, min(g.bias_max, self.bias))
            w = (w_gyro - self.bias) * g.scale
            self.source = GYRO
        # Point milieu : sur un arc, integrer au cap du milieu du pas plutot
        # qu'a celui du debut divise l'erreur par le nombre de pas.
        middle = self.yaw + w * dt / 2
        self.x += v * math.cos(middle) * dt
        self.y += v * math.sin(middle) * dt
        self.yaw = math.atan2(math.sin(self.yaw + w * dt), math.cos(self.yaw + w * dt))

    @property
    def pose(self) -> tuple[float, float, float]:
        return self.x, self.y, self.yaw


def yaw_rate(rot: tuple[tuple[float, float, float], ...],
             gyro: tuple[float, float, float]) -> float:
    """Vitesse de lacet du ROBOT, a partir du gyromètre exprime dans SON repere.

    `rot` est la rotation du repere de l'IMU vers celui de la base, telle que
    TF la donne : w_base = rot . w_imu, et le lacet en est la composante z. Seule
    la troisieme ligne sert.

    Pourquoi ne pas lire `gyro.z` directement, ce que faisait ce noeud : parce
    que l'IMU n'est pas forcement posee a plat et dans le bon sens. Sur ce robot
    elle est montee A L'ENVERS -- l'URDF le decrit fidelement, et le z du capteur
    pointe donc vers le bas. Lire z brut donnait un cap qui tournait a l'envers,
    distances justes : mesure en simulation, commande +0.5 rad/s, robot tournant
    bien de +26.3 deg, gyromètre annoncant -0.439 rad/s.

    Passer par TF rend le montage transparent, dans les deux sens : une IMU
    remontee autrement ne demande plus qu'un export correct, et aucun code ne
    porte de signe code en dur.
    """
    return sum(rot[2][k] * gyro[k] for k in range(3))


def rotation_from_quaternion(x: float, y: float, z: float, w: float):
    """Matrice de rotation d'un quaternion TF, en lignes."""
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


def usable(gyro: tuple[float, float, float] | None,
           accel: tuple[float, float, float] | None) -> bool:
    """Un echantillon IMU est exploitable s'il n'est pas une trame de zeros.

    Le firmware met gyro ET accel a zero quand la lecture I2C echoue ; or sous
    gravite `accel` ne peut pas etre nul. Six zeros exacts = panne, pas repos.
    """
    if gyro is None or accel is None:
        return False
    return any(v != 0.0 for v in accel) or any(v != 0.0 for v in gyro)
