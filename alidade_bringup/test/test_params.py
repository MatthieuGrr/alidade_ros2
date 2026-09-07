"""Les gabarits de config/ se rendent tous depuis la revision v1, sans marqueur oublie."""
from pathlib import Path

import pytest
import yaml

from alidade_bringup import params

CONFIGS = ['controllers.yaml', 'nav2.yaml', 'slam.yaml', 'rplidar.yaml', 'camera.yaml',
           'telemetry.yaml', 'diagnostics.yaml']


@pytest.fixture(scope='module')
def revision():
    return params.load_revision('v1')


@pytest.mark.parametrize('filename', CONFIGS)
def test_render(filename, revision):
    rendered = Path(params.render_config(filename, revision, use_sim_time=False))
    text = rendered.read_text()
    assert '${' not in text
    yaml.safe_load(text)


def test_kinematics_reach_controllers(revision):
    rendered = yaml.safe_load(Path(params.render_config('controllers.yaml', revision, True)).read_text())
    ddc = rendered['diff_drive_controller']['ros__parameters']
    assert ddc['wheel_separation'] == revision['kinematics']['wheel_separation']
    assert ddc['wheel_radius'] == revision['kinematics']['wheel_radius']
    assert rendered['controller_manager']['ros__parameters']['use_sim_time'] is True


def test_missing_placeholder_fails(tmp_path, revision):
    template = tmp_path / 'bad.yaml'
    template.write_text('x: ${kinematics.inconnu}\n')
    with pytest.raises(KeyError):
        params.render(template, revision)
