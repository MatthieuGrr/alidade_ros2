"""Les lecteurs systeme et la mise en forme des diagnostics, sans ROS ni Pi."""
from alidade_bringup import telemetry

TH = dict(temp_warn_c=75.0, temp_error_c=85.0, rail_5v_warn_v=4.80, cpu_warn_percent=85.0,
          mem_warn_percent=85.0, disk_warn_percent=85.0, wifi_warn_dbm=-75.0, rate_warn_ratio=0.8)


def by_name(statuses):
    return {s.name.split(': ', 1)[1]: s for s in statuses}


def test_throttled_flags():
    flags = telemetry.parse_throttled('throttled=0x50005\n')
    assert flags['now'] == ['sous-tension', 'throttling actif']
    assert flags['past'] == ['sous-tension survenue', 'throttling survenu']
    assert telemetry.parse_throttled('throttled=0x0')['now'] == []


def test_pmic_parsing():
    text = '3V7_WL_SW_A current(0)=0.06000000A\nEXT5V_V volt(24)=4.95800000V\n'
    values = telemetry.parse_pmic(text)
    assert values['EXT5V_V'] == 4.958
    assert values['3V7_WL_SW_A'] == 0.06


def test_wireless_parsing():
    text = ('Inter-| sta-|   Quality        |   Discarded packets\n'
            ' face | tus | link level noise |  nwid  crypt   frag\n'
            ' wlan0: 0000   70.  -40.  -256        0      0      0\n')
    wl = telemetry.parse_wireless(text)
    assert wl == {'iface': 'wlan0', 'link': 70.0, 'level_dbm': -40.0}
    assert telemetry.parse_wireless('Inter-|\n face |\n') is None


def test_levels_from_thresholds():
    sample = {
        'temp_c': 80.0,
        'throttled': telemetry.parse_throttled('throttled=0x0'),
        'pmic': {'EXT5V_V': 4.70},
        'wireless': {'iface': 'wlan0', 'link': 50.0, 'level_dbm': -80.0},
        'ntp': False,
        'system': {'cpu_percent': 20.0, 'cpu_per_core': [20.0], 'load_1': 0.5, 'mem_percent': 30.0,
                   'mem_available_mb': 5000.0, 'disk_percent': 90.0, 'disk_free_gb': 3.0, 'uptime_s': 100.0},
    }
    st = by_name(telemetry.statuses(sample, TH))
    assert st['Temperature'].level == telemetry.WARN
    assert st['Alimentation'].level == telemetry.WARN
    assert st['CPU'].level == telemetry.OK
    assert st['Disque'].level == telemetry.WARN
    assert st['WiFi'].level == telemetry.WARN
    assert st['Horloge'].level == telemetry.WARN

    sample['temp_c'] = 90.0
    assert by_name(telemetry.statuses(sample, TH))['Temperature'].level == telemetry.ERROR
    sample['temp_c'] = 50.0
    sample['throttled'] = telemetry.parse_throttled('throttled=0x1')
    assert by_name(telemetry.statuses(sample, TH))['Temperature'].level == telemetry.ERROR


def test_outside_pi_is_not_an_error():
    sample = {'temp_c': None, 'throttled': None, 'pmic': None, 'wireless': None, 'ntp': True, 'system': {}}
    st = by_name(telemetry.statuses(sample, TH))
    assert st['Alimentation'].level == telemetry.OK
    assert st['Temperature'].level == telemetry.WARN
    assert 'CPU' not in st


def test_rate_status():
    assert telemetry.rate_status('/scan', None, 10.0, 0.8).level == telemetry.ERROR
    assert telemetry.rate_status('/scan', 7.0, 10.0, 0.8).level == telemetry.WARN
    assert telemetry.rate_status('/scan', 9.9, 10.0, 0.8).level == telemetry.OK
    assert telemetry.rate_status('/scan', 9.9, 10.0, 0.8).name == 'system_monitor: Cadence /scan'
    assert telemetry.rate_status('/camera/image', None, 15.0, 0.8, optional=True).level == telemetry.WARN
