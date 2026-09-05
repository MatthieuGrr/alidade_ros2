#!/usr/bin/env python3
"""Faux Pico sur un pseudo-terminal, pour eprouver alidade_hardware sans materiel.

Rejoue le comportement de diffdrive_pico_firmware : parseur strict, trames S
a 50 Hz avec checksum, chien de garde de 300 ms, roues du premier ordre avec
zone morte, encodeurs quantifies. Le modele des roues et les gains sont ceux
identifies sur le robot par tools/tune_pid.py du firmware.

    ros2 run alidade_hardware faux_pico.py                 # affiche le port cree
    ros2 run alidade_hardware faux_pico.py --link /tmp/pico
    ros2 launch alidade_bringup robot.launch.py lidar:=false pico_port:=/tmp/pico

Le port est un esclave de pty : il se comporte comme un /dev/ttyACM*, y compris
pour le mode brut. Le faux Pico garde son cote ouvert, donc le port survit aux
ouvertures et fermetures successives du client.
"""
from __future__ import annotations

import argparse
import functools
import math
import operator
import os
import select
import termios
import time
import tty

FRAME_HZ = 50
WATCHDOG_S = 0.300
DUTY_MAX = 1000


class Wheel:
    """Premier ordre avec zone morte, encodeur quantifie : x en ticks."""

    def __init__(self, gain: float, tau: float, dead: float):
        self.gain, self.tau, self.dead = gain, tau, dead
        self.v = 0.0
        self.x = 0.0

    def step(self, duty: float, dt: float) -> None:
        mag = max(abs(duty) - self.dead, 0.0)
        target = self.gain * mag * (1 if duty >= 0 else -1)
        e = math.exp(-dt / self.tau)
        self.x += target * dt + (self.v - target) * self.tau * (1 - e)
        self.v = target + (self.v - target) * e

    @property
    def ticks(self) -> int:
        return int(self.x)


class PID:
    """Transcription de pid.c : anticipation, integrale bornee, derivee sur la mesure."""

    def __init__(self, kp=0.1749, ki=0.8074, kd=0.0, kff=0.266):
        self.kp, self.ki, self.kd, self.kff = kp, ki, kd, kff
        self.reset()

    def reset(self) -> None:
        self.integ = 0.0
        self.prev = 0.0
        self.primed = False

    def update(self, sp: float, meas: float, dt: float) -> float:
        err = sp - meas
        d = -self.kd * (meas - self.prev) / dt if (self.primed and self.kd) else 0.0
        self.prev, self.primed = meas, True
        it = self.integ + err * dt
        out = self.kff * sp + self.kp * err + self.ki * it + d
        if out > DUTY_MAX:
            out = DUTY_MAX
            if err > 0:
                it = self.integ
        elif out < -DUTY_MAX:
            out = -DUTY_MAX
            if err < 0:
                it = self.integ
        self.integ = max(min(it, DUTY_MAX / self.ki), -DUTY_MAX / self.ki) if self.ki > 0 else 0.0
        return out


def checksum(payload: str) -> int:
    return functools.reduce(operator.xor, payload.encode(), 0)


class FauxPico:
    def __init__(self, imu: bool, verbose: bool):
        self.wheels = [Wheel(3.800, 0.066, 66), Wheel(3.731, 0.058, 77)]
        self.pid = [PID(), PID()]
        self.mode = 'stop'
        self.setpoint = [0, 0]
        self.open_loop = [0, 0]
        self.offset = [0, 0]
        self.test_mode = 0
        self.last_cmd = None
        self.t0 = time.monotonic()
        self.imu = imu
        self.verbose = verbose
        self.velocity = [0.0, 0.0]
        self.prev_ticks = [0, 0]
        self.rejected = 0

    def command(self, line: str) -> None:
        parts = line.split()
        if not parts:
            return
        ok = False
        try:
            c = parts[0]
            if c == 'v' and len(parts) == 3:
                self.setpoint = [int(parts[1]), int(parts[2])]
                self.mode = 'vel'
                ok = True
            elif c == 'o' and len(parts) == 3:
                self.open_loop = [max(-DUTY_MAX, min(DUTY_MAX, int(parts[1]))),
                                  max(-DUTY_MAX, min(DUTY_MAX, int(parts[2])))]
                self.mode = 'ol'
                ok = True
            elif c == 's' and len(parts) == 1:
                self.stop()
                ok = True
            elif c == 'r' and len(parts) == 1:
                self.offset = [w.ticks for w in self.wheels]
                ok = True
            elif c == 'k' and len(parts) == 4:
                for p in self.pid:
                    p.kp, p.ki, p.kd = float(parts[1]), float(parts[2]), float(parts[3])
                ok = True
            elif c == 't' and len(parts) == 2:
                self.test_mode = int(parts[1])
                ok = True
        except ValueError:
            ok = False
        if ok:
            self.last_cmd = time.monotonic()
        else:
            self.rejected += 1
            if self.verbose:
                print(f'rejete : {line!r}')

    def stop(self) -> None:
        self.mode = 'stop'
        self.setpoint = [0, 0]
        self.open_loop = [0, 0]
        for p in self.pid:
            p.reset()

    def step(self, dt: float) -> None:
        # Chien de garde : sans commande valide depuis 300 ms, PWM a zero.
        if self.mode != 'stop' and self.test_mode == 0 and \
                (self.last_cmd is None or time.monotonic() - self.last_cmd > WATCHDOG_S):
            if self.verbose:
                print('chien de garde : moteurs coupes')
            self.stop()
        for i, w in enumerate(self.wheels):
            if self.mode == 'vel':
                duty = self.pid[i].update(self.setpoint[i], self.velocity[i], dt)
            elif self.mode == 'ol':
                duty = self.open_loop[i]
            else:
                duty = 0.0
            w.step(duty, dt)
            # Vitesse mesuree comme le firmware : ticks vus sur la periode,
            # filtres au premier ordre (alpha 0.5).
            raw = (w.ticks - self.prev_ticks[i]) / dt
            self.prev_ticks[i] = w.ticks
            self.velocity[i] = 0.5 * raw + 0.5 * self.velocity[i]

    def frame(self) -> bytes:
        t_ms = int((time.monotonic() - self.t0) * 1000)
        enc = [w.ticks - o for w, o in zip(self.wheels, self.offset)]
        vel = [int(v) for v in self.velocity]
        # Robot pose a plat : gravite sur +Z, 100 LSB par m/s^2 ; gyroscope
        # deduit de la vitesse differentielle, 16 LSB par deg/s, avec un
        # rayon et un entraxe fictifs juste pour donner un ordre de grandeur.
        if self.imu:
            yaw_rate_dps = (vel[1] - vel[0]) * 0.02
            gyro = [0, 0, int(yaw_rate_dps * 16)]
            accel = [0, 0, 981]
        else:
            gyro, accel = [0, 0, 0], [0, 0, 0]
        payload = f'S {t_ms} {enc[0]} {enc[1]} {vel[0]} {vel[1]} ' \
                  f'{gyro[0]} {gyro[1]} {gyro[2]} {accel[0]} {accel[1]} {accel[2]}'
        return f'{payload} {checksum(payload):02X}\n'.encode()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--link', help='lien symbolique a creer vers le port, ex. /tmp/pico')
    ap.add_argument('--no-imu', action='store_true', help='simule un BNO055 absent (champs a 0)')
    ap.add_argument('--corrupt', type=float, default=0.0,
                    help='fraction de trames a corrompre, pour tester le rejet cote Pi')
    ap.add_argument('-v', '--verbose', action='store_true')
    args = ap.parse_args()

    master, slave = os.openpty()
    tty.setraw(slave)
    attrs = termios.tcgetattr(slave)
    attrs[3] &= ~termios.ECHO
    termios.tcsetattr(slave, termios.TCSANOW, attrs)
    port = os.ttyname(slave)
    if args.link:
        if os.path.islink(args.link):
            os.unlink(args.link)
        os.symlink(port, args.link)
    print(f'faux Pico sur {port}' + (f' (lien {args.link})' if args.link else ''), flush=True)

    pico = FauxPico(imu=not args.no_imu, verbose=args.verbose)
    period = 1.0 / FRAME_HZ
    next_frame = time.monotonic()
    pending = b''
    sent = 0
    corrupted = 0
    try:
        while True:
            timeout = max(0.0, next_frame - time.monotonic())
            ready, _, _ = select.select([master], [], [], timeout)
            if ready:
                try:
                    pending += os.read(master, 4096)
                except OSError:
                    pass
                while b'\n' in pending:
                    line, pending = pending.split(b'\n', 1)
                    pico.command(line.decode('ascii', 'replace').strip())
            now = time.monotonic()
            if now >= next_frame:
                pico.step(period)
                frame = pico.frame()
                if args.corrupt and (sent % max(1, int(1 / args.corrupt)) == 0):
                    frame = frame[:-3] + b'00\n'
                    corrupted += 1
                try:
                    os.write(master, frame)
                except OSError:
                    pass
                sent += 1
                next_frame += period
                if args.verbose and sent % FRAME_HZ == 0:
                    print(f'{frame.decode().strip()}  mode={pico.mode} sp={pico.setpoint} '
                          f'rejets={pico.rejected} corrompues={corrupted}', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        if args.link and os.path.islink(args.link):
            os.unlink(args.link)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
