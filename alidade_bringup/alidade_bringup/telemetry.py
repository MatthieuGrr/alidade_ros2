"""Mesures systeme du Pi 5 et mise en forme en diagnostics ROS.

Fonctions pures, sans rclpy, pour etre testables : chaque lecteur rend un
dict, ou None quand la source n'existe pas (vcgencmd hors du Pi, pas de WiFi),
et statuses() les convertit en DiagnosticStatus selon les seuils de la
revision materielle. Le noeud system_monitor.py ne fait qu'appeler et publier.
"""
from __future__ import annotations

import re
import shutil
import subprocess
import time
from pathlib import Path

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

try:
    import psutil
except ImportError:  # pragma: no cover - le paquet est dans apt-packages.txt
    psutil = None

OK, WARN, ERROR, STALE = (DiagnosticStatus.OK, DiagnosticStatus.WARN,
                          DiagnosticStatus.ERROR, DiagnosticStatus.STALE)

# Bits de `vcgencmd get_throttled`. Les bits 16..19 sont les memes evenements,
# memorises depuis le boot.
THROTTLE_FLAGS = {
    0: 'sous-tension',
    1: 'frequence ARM bridee',
    2: 'throttling actif',
    3: 'limite thermique douce',
    16: 'sous-tension survenue',
    17: 'frequence ARM bridee (survenu)',
    18: 'throttling survenu',
    19: 'limite thermique douce (survenue)',
}

THERMAL = Path('/sys/class/thermal/thermal_zone0/temp')
WIRELESS = Path('/proc/net/wireless')
TIMESYNC = Path('/run/systemd/timesync/synchronized')


# --------------------------------------------------------------------------- lecteurs

def vcgencmd(*args: str) -> str | None:
    if shutil.which('vcgencmd') is None:
        return None
    try:
        out = subprocess.run(['vcgencmd', *args], capture_output=True, text=True, timeout=2.0)
    except (OSError, subprocess.SubprocessError):
        return None
    return out.stdout if out.returncode == 0 else None


def parse_throttled(text: str) -> dict:
    """'throttled=0x50005' -> drapeaux courants et passes."""
    match = re.search(r'0x([0-9a-fA-F]+)', text)
    value = int(match.group(1), 16) if match else 0
    return {
        'raw': f'0x{value:x}',
        'now': [name for bit, name in THROTTLE_FLAGS.items() if bit < 16 and value >> bit & 1],
        'past': [name for bit, name in THROTTLE_FLAGS.items() if bit >= 16 and value >> bit & 1],
    }


def parse_pmic(text: str) -> dict[str, float]:
    """Sortie de `vcgencmd pmic_read_adc` : 'EXT5V_V volt(24)=4.95800000V'."""
    values = {}
    for line in text.splitlines():
        match = re.match(r'\s*(\S+)\s+(?:volt|current)\(\d+\)=([0-9.]+)[VA]', line)
        if match:
            values[match.group(1)] = float(match.group(2))
    return values


def parse_wireless(text: str) -> dict | None:
    """Premiere interface de /proc/net/wireless : qualite de lien et niveau en dBm."""
    for line in text.splitlines()[2:]:
        parts = line.split()
        if len(parts) < 4:
            continue
        try:
            return {
                'iface': parts[0].rstrip(':'),
                'link': float(parts[2].rstrip('.')),
                'level_dbm': float(parts[3].rstrip('.')),
            }
        except ValueError:
            continue
    return None


def read_thermal() -> float | None:
    try:
        return int(THERMAL.read_text().strip()) / 1000.0
    except (OSError, ValueError):
        return None


def read_wireless() -> dict | None:
    try:
        return parse_wireless(WIRELESS.read_text())
    except OSError:
        return None


def ntp_synchronized() -> bool:
    return TIMESYNC.exists()


def read_system() -> dict:
    """Charge, memoire, disque, uptime via psutil (cpu_percent est un delta
    depuis l'appel precedent : appeler read_system() a cadence fixe)."""
    if psutil is None:
        return {}
    mem = psutil.virtual_memory()
    disk = psutil.disk_usage('/')
    return {
        'cpu_percent': psutil.cpu_percent(interval=None),
        'cpu_per_core': psutil.cpu_percent(interval=None, percpu=True),
        'load_1': psutil.getloadavg()[0],
        'mem_percent': mem.percent,
        'mem_available_mb': mem.available / 1e6,
        'disk_percent': disk.percent,
        'disk_free_gb': disk.free / 1e9,
        'uptime_s': time.time() - psutil.boot_time(),
    }


def sample() -> dict:
    """Toutes les sources d'un coup ; les absentes valent None."""
    throttled = vcgencmd('get_throttled')
    pmic = vcgencmd('pmic_read_adc')
    return {
        'temp_c': read_thermal(),
        'throttled': parse_throttled(throttled) if throttled else None,
        'pmic': parse_pmic(pmic) if pmic else None,
        'wireless': read_wireless(),
        'ntp': ntp_synchronized(),
        'system': read_system(),
    }


# --------------------------------------------------------------------------- statuts

def _status(name: str, level: int, message: str, values: dict, hardware_id: str = 'pi5') -> DiagnosticStatus:
    st = DiagnosticStatus(name=name, level=level, message=message, hardware_id=hardware_id)
    st.values = [KeyValue(key=k, value=(f'{v:.2f}' if isinstance(v, float) else str(v)))
                 for k, v in values.items()]
    return st


def statuses(s: dict, th: dict, prefix: str = 'system_monitor') -> list[DiagnosticStatus]:
    out = []

    # Temperature et throttling ensemble : c'est la meme histoire.
    temp = s.get('temp_c')
    thr = s.get('throttled')
    values = {'temperature (C)': temp if temp is not None else 'n/a'}
    if thr:
        values['throttled'] = thr['raw']
        values['en cours'] = ', '.join(thr['now']) or 'aucun'
        values['depuis le boot'] = ', '.join(thr['past']) or 'aucun'
    if temp is None:
        level, msg = WARN, 'temperature indisponible'
    elif temp >= th['temp_error_c'] or (thr and thr['now']):
        level, msg = ERROR, f'{temp:.0f} C' + (' ; ' + ', '.join(thr['now']) if thr and thr['now'] else '')
    elif temp >= th['temp_warn_c']:
        level, msg = WARN, f'{temp:.0f} C'
    else:
        level, msg = OK, f'{temp:.0f} C'
    out.append(_status(f'{prefix}: Temperature', level, msg, values))

    # Alimentation : rail 5 V d'entree et rails internes du PMIC.
    pmic = s.get('pmic')
    if pmic:
        ext5v = pmic.get('EXT5V_V')
        values = {k: v for k, v in sorted(pmic.items()) if k.endswith('_V') or k.endswith('_A')}
        if ext5v is None:
            level, msg = WARN, 'EXT5V_V absent'
        elif ext5v < th['rail_5v_warn_v']:
            level, msg = WARN, f'5 V a {ext5v:.2f} V'
        else:
            level, msg = OK, f'5 V a {ext5v:.2f} V'
        out.append(_status(f'{prefix}: Alimentation', level, msg, values))
    else:
        out.append(_status(f'{prefix}: Alimentation', OK, 'pas de PMIC (hors Pi)', {}))

    # CPU, memoire, disque.
    sysm = s.get('system') or {}
    if sysm:
        cpu = sysm['cpu_percent']
        values = {'cpu (%)': cpu, 'par coeur (%)': ' '.join(f'{c:.0f}' for c in sysm['cpu_per_core']),
                  'charge 1 min': sysm['load_1'], 'uptime (s)': int(sysm['uptime_s'])}
        level = WARN if cpu >= th['cpu_warn_percent'] else OK
        out.append(_status(f'{prefix}: CPU', level, f'{cpu:.0f} %', values))

        mem = sysm['mem_percent']
        level = WARN if mem >= th['mem_warn_percent'] else OK
        out.append(_status(f'{prefix}: Memoire', level, f'{mem:.0f} %',
                           {'utilisee (%)': mem, 'disponible (MB)': sysm['mem_available_mb']}))

        disk = sysm['disk_percent']
        level = WARN if disk >= th['disk_warn_percent'] else OK
        out.append(_status(f'{prefix}: Disque', level, f'{disk:.0f} %',
                           {'utilise (%)': disk, 'libre (GB)': sysm['disk_free_gb']}))

    # WiFi.
    wl = s.get('wireless')
    if wl:
        level = WARN if wl['level_dbm'] <= th['wifi_warn_dbm'] else OK
        out.append(_status(f'{prefix}: WiFi', level, f"{wl['level_dbm']:.0f} dBm",
                           {'interface': wl['iface'], 'niveau (dBm)': wl['level_dbm'], 'qualite': wl['link']}))
    else:
        out.append(_status(f'{prefix}: WiFi', WARN, 'pas d interface sans fil', {}))

    # Horloge : le bringup en depend (voir alidade.service).
    ntp = s.get('ntp')
    out.append(_status(f'{prefix}: Horloge', OK if ntp else WARN,
                       'synchronisee' if ntp else 'non synchronisee', {'ntp': ntp}))
    return out


def rate_status(topic: str, hz: float | None, expected: float, ratio: float,
                prefix: str = 'system_monitor', optional: bool = False) -> DiagnosticStatus:
    """Cadence mesuree d'un topic face a la cadence attendue. Un capteur
    optionnel (camera) absent vaut un avertissement, pas une erreur."""
    values = {'attendue (Hz)': expected, 'mesuree (Hz)': hz if hz is not None else 0.0,
              'optionnel': optional}
    if hz is None or hz <= 0.0:
        level, msg = (WARN, 'absent (optionnel)') if optional else (ERROR, 'aucun message')
    elif hz < ratio * expected:
        level, msg = WARN, f'{hz:.1f} Hz pour {expected:.0f} attendus'
    else:
        level, msg = OK, f'{hz:.1f} Hz'
    return _status(f'{prefix}: Cadence {topic}', level, msg, values, hardware_id=topic)
