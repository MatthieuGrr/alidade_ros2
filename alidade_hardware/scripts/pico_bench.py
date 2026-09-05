#!/usr/bin/env python3
"""Banc d'essai du Pico, sans ros2_control : valide le protocole et le sens
des roues avant de brancher diff_drive_controller.

    pico_bench.py monitor                   # trames, rejets, compteurs, IMU
    pico_bench.py wheels                    # sens de rotation de chaque roue
    pico_bench.py drive 500 500 3           # consigne en ticks/s pendant 3 s
    pico_bench.py stop

Options : --port /dev/pico, --yaml <revision> pour convertir les ticks en
metres (sinon la revision v1 est cherchee dans alidade_description).
Le banc emet la consigne a 20 Hz tant qu'il roule : le chien de garde de
300 ms du firmware coupe les moteurs des qu'on l'arrete.
"""
from __future__ import annotations

import argparse
import functools
import math
import operator
import sys
import threading
import time
from pathlib import Path

import serial
import yaml


def verify(line: str) -> bool:
    try:
        payload, chk = line.rstrip('\n').rsplit(' ', 1)
    except ValueError:
        return False
    return payload.startswith('S ') and \
        functools.reduce(operator.xor, payload.encode(), 0) == int(chk, 16)


class Pico:
    def __init__(self, port: str):
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        self.ser.reset_input_buffer()
        self.lock = threading.Lock()
        self.frame = None
        self.received = 0
        self.rejected = 0
        self.last_at = None
        self.command = None
        self.running = True
        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._sender, daemon=True).start()

    def _reader(self) -> None:
        while self.running:
            raw = self.ser.readline()
            if not raw:
                continue
            line = raw.decode('ascii', 'replace')
            if not line.endswith('\n'):
                continue
            with self.lock:
                if verify(line):
                    f = line.split()
                    self.frame = [int(v) for v in f[1:12]]
                    self.received += 1
                    self.last_at = time.monotonic()
                else:
                    self.rejected += 1

    def _sender(self) -> None:
        while self.running:
            with self.lock:
                cmd = self.command
            if cmd:
                self.ser.write(cmd)
            time.sleep(0.05)

    def velocity(self, left: int, right: int) -> None:
        with self.lock:
            self.command = f'v {left} {right}\n'.encode()

    def stop(self) -> None:
        with self.lock:
            self.command = None
        self.ser.write(b's\n')

    def encoders(self):
        with self.lock:
            return (self.frame[1], self.frame[2]) if self.frame else None

    def close(self) -> None:
        self.running = False
        self.stop()
        self.ser.close()


def load_revision(path: str | None) -> dict | None:
    if path:
        return yaml.safe_load(Path(path).read_text())
    try:
        from ament_index_python.packages import get_package_share_directory
        p = Path(get_package_share_directory('alidade_description')) / 'config' / 'alidade_v1.yaml'
        return yaml.safe_load(p.read_text())
    except Exception:
        return None


def wait_frame(pico: Pico, seconds: float = 2.0) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if pico.frame:
            return
        time.sleep(0.05)
    sys.exit(f'aucune trame S valide en {seconds:.0f} s ({pico.rejected} rejetees) : '
             'port, cablage ou firmware')


def monitor(pico: Pico, hw: dict | None) -> None:
    wait_frame(pico)
    accel_scale = 1.0 / hw['imu']['accel_lsb_per_m_s2'] if hw else 0.01
    gyro_scale = 1.0 / hw['imu']['gyro_lsb_per_deg_s'] if hw else 1 / 16
    last_n, last_t = pico.received, time.monotonic()
    print('t_ms      enc_l    enc_r    vel_l  vel_r   gyro (deg/s)          accel (m/s2)         trames/s  rejets')
    try:
        while True:
            time.sleep(0.2)
            with pico.lock:
                f, n, rej = pico.frame, pico.received, pico.rejected
            now = time.monotonic()
            rate = (n - last_n) / (now - last_t)
            last_n, last_t = n, now
            g = [v * gyro_scale for v in f[5:8]]
            a = [v * accel_scale for v in f[8:11]]
            print(f'{f[0]:<9} {f[1]:<8} {f[2]:<8} {f[3]:<6} {f[4]:<6} '
                  f'{g[0]:+6.1f} {g[1]:+6.1f} {g[2]:+6.1f}   '
                  f'{a[0]:+6.2f} {a[1]:+6.2f} {a[2]:+6.2f}   {rate:5.1f}     {rej}')
    except KeyboardInterrupt:
        pass


def wheels(pico: Pico) -> None:
    """Chaque roue seule, en avant : ses ticks doivent croitre et l'autre roue
    rester immobile. Un compteur qui decroit = encodeur ou moteur inverse ;
    la mauvaise roue qui bouge = cables gauche/droite croises."""
    wait_frame(pico)
    print('robot SURELEVE, roues dans le vide')
    ok = True
    for i, name in enumerate(('gauche', 'droite')):
        before = pico.encoders()
        pico.velocity(300 if i == 0 else 0, 300 if i == 1 else 0)
        time.sleep(2.0)
        pico.stop()
        time.sleep(0.5)
        after = pico.encoders()
        delta = [after[k] - before[k] for k in range(2)]
        moved, other = delta[i], delta[1 - i]
        verdict = 'OK' if moved > 200 and abs(other) < 50 else 'PROBLEME'
        ok &= verdict == 'OK'
        print(f'roue {name:7s} : consigne +300 ticks/s pendant 2 s -> '
              f'elle {moved:+d} ticks, autre {other:+d} ticks   {verdict}')
    print('sens des roues coherent' if ok else
          'verifier MOT_x_INVERT / ENC_x_INVERT (mode t 3 du firmware) ou le cablage')


def drive(pico: Pico, hw: dict | None, left: int, right: int, seconds: float) -> None:
    wait_frame(pico)
    before = pico.encoders()
    t0 = time.monotonic()
    pico.velocity(left, right)
    time.sleep(seconds)
    pico.stop()
    elapsed = time.monotonic() - t0
    time.sleep(0.5)
    after = pico.encoders()
    delta = [after[k] - before[k] for k in range(2)]
    print(f'{elapsed:.2f} s : gauche {delta[0]:+d} ticks, droite {delta[1]:+d} ticks')
    if hw:
        k = hw['kinematics']
        m_per_tick = 2 * math.pi * k['wheel_radius'] / k['counts_per_rev']
        dl, dr = delta[0] * m_per_tick, delta[1] * m_per_tick
        dist = (dl + dr) / 2
        yaw = (dr - dl) / k['wheel_separation']
        print(f'soit gauche {dl:+.3f} m, droite {dr:+.3f} m : avance {dist:+.3f} m, '
              f'rotation {math.degrees(yaw):+.1f} deg')
        print(f'a verifier au metre : {abs(dist):.3f} m attendus')


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default='/dev/pico')
    ap.add_argument('--yaml', help='fichier de revision materielle')
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('monitor')
    sub.add_parser('wheels')
    d = sub.add_parser('drive')
    d.add_argument('left', type=int)
    d.add_argument('right', type=int)
    d.add_argument('seconds', type=float)
    sub.add_parser('stop')
    args = ap.parse_args()

    hw = load_revision(args.yaml)
    pico = Pico(args.port)
    try:
        if args.cmd == 'monitor':
            monitor(pico, hw)
        elif args.cmd == 'wheels':
            wheels(pico)
        elif args.cmd == 'drive':
            drive(pico, hw, args.left, args.right, args.seconds)
        elif args.cmd == 'stop':
            pico.stop()
    finally:
        pico.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
