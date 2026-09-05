"""Rendu des parametres ROS depuis la revision materielle.

Les fichiers de config/ sont des gabarits : chaque constante physique y est un
marqueur ${section.cle} resolu ici depuis alidade_<revision>.yaml, et
${use_sim_time} vient du launch. Un marqueur non resolu fait echouer le
lancement : un gabarit oublie ne roule jamais silencieusement avec un zero.
"""
from __future__ import annotations

import re
import subprocess
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

PLACEHOLDER = re.compile(r"\$\{([a-zA-Z0-9_.]+)\}")


def description_share() -> Path:
    return Path(get_package_share_directory('alidade_description'))


def bringup_share() -> Path:
    return Path(get_package_share_directory('alidade_bringup'))


def load_revision(revision: str) -> dict:
    path = description_share() / 'config' / f'alidade_{revision}.yaml'
    if not path.exists():
        raise FileNotFoundError(f"revision materielle inconnue : {path}")
    return yaml.safe_load(path.read_text())


def _lookup(values: dict, dotted: str):
    node = values
    for key in dotted.split('.'):
        if not isinstance(node, dict) or key not in node:
            raise KeyError(dotted)
        node = node[key]
    return node


def _scalar(value) -> str:
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, (int, float, str)):
        return str(value)
    raise TypeError(f"valeur non scalaire pour un gabarit : {value!r}")


def render(template: Path, revision: dict, extra: dict | None = None, name: str | None = None) -> str:
    """Rend un gabarit dans un fichier temporaire et retourne son chemin."""
    values = dict(revision)
    values.update(extra or {})
    text = template.read_text()

    missing = []

    def substitute(match: re.Match) -> str:
        try:
            return _scalar(_lookup(values, match.group(1)))
        except KeyError:
            missing.append(match.group(1))
            return match.group(0)

    rendered = PLACEHOLDER.sub(substitute, text)
    if missing:
        raise KeyError(f"{template.name} : marqueurs non resolus : {', '.join(sorted(set(missing)))}")

    out_dir = Path(tempfile.mkdtemp(prefix='alidade_'))
    out = out_dir / (name or template.name)
    out.write_text(rendered)
    return str(out)


def render_config(filename: str, revision: dict, use_sim_time: bool) -> str:
    return render(bringup_share() / 'config' / filename, revision, {'use_sim_time': use_sim_time})


def robot_description(revision: str, target: str, use_mock_hardware: bool = False,
                      pico_port: str = '/dev/pico', controllers_file: str = '') -> str:
    """URDF final, produit par xacro avec les arguments de la cible."""
    xacro = description_share() / 'urdf' / 'alidade.urdf.xacro'
    cmd = [
        'xacro', str(xacro),
        f'hardware_revision:={revision}',
        f'target:={target}',
        f'use_mock_hardware:={"true" if use_mock_hardware else "false"}',
        f'pico_port:={pico_port}',
        f'controllers_file:={controllers_file}',
    ]
    return subprocess.run(cmd, capture_output=True, text=True, check=True).stdout


def as_bool(value: str) -> bool:
    return str(value).lower() in ('true', '1', 'yes')
