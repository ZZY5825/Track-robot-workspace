"""Contract checks for the beginner-facing modular bringup quick start."""

from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[4]
GUIDE = (
    WORKSPACE_ROOT
    / 'docs'
    / 'guides'
    / 'semantic-search'
    / 'phase2-recording-and-evaluation.md'
)


def _guide():
    return GUIDE.read_text(encoding='utf-8')


def _bash_block_after(text, marker):
    remainder = text.split(marker, 1)[1]
    return remainder.split('```bash', 1)[1].split('```', 1)[0]


def test_quick_start_documents_fixed_domain_and_all_control_commands():
    text = _guide()

    assert 'ROS_DOMAIN_ID=20' in text
    for command in ('doctor', 'start', 'status', 'query', 'test', 'stop'):
        assert 'semantic_search_ctl {}'.format(command) in text
    assert 'semantic_search_ctl doctor phase1' in text
    assert 'semantic_search_ctl doctor phase2' in text


def test_each_second_terminal_block_is_independently_initialized():
    text = _guide()
    phase1 = _bash_block_after(text, 'In a second Domain 20 terminal')
    phase2 = _bash_block_after(text, 'In a second terminal')

    for block in (phase1, phase2):
        assert 'source /opt/ros/foxy/setup.bash' in block
        assert 'source ~/track_robot_ws/install/setup.bash' in block
        assert 'export ROS_DOMAIN_ID=20' in block
    assert (
        'EXTRINSIC=~/track_robot_ws/config/'
        'camera_extrinsic.measured.yaml'
    ) in phase2
    assert phase2.index('EXTRINSIC=') < phase2.index('$EXTRINSIC')


def test_quick_start_documents_hardware_and_calibration_safety_contracts():
    text = _guide()

    assert '--hardware auto' in text
    assert '--hardware external' in text
    assert '--extrinsic-mode measured' in text
    assert '--extrinsic-mode prototype' in text
    assert '--allow-degraded' in text
    assert '/cmd_vel' in text
    assert 'does not start' in text
    assert 'reuses required modules that already have a publisher' in text
    assert 'Readiness then validates' in text
    assert 'reuses healthy required sensor publishers' not in text
    assert 'reuse a healthy camera' not in text


def test_quick_start_documents_reports_statuses_and_exit_codes():
    text = _guide()

    assert '~/.ros/track_robot_semantic_search/reports/' in text
    for status in ('PASS', 'NOT READY', 'DEGRADED', 'FAIL'):
        assert status in text
    for exit_code in ('`0`', '`2`', '`3`', '`4`', '`130`'):
        assert exit_code in text


def test_quick_start_keeps_evidence_outside_raw_rosbags():
    text = _guide()

    assert 'artifacts/semantic_search/manifests/' in text
    assert 'artifacts/semantic_search/annotations/' in text
    assert 'artifacts/semantic_search/reports/' in text
    assert 'rosbags/semantic_search/manifests/' not in text
    assert 'rosbags/semantic_search/annotations/' not in text
    assert 'rosbags/semantic_search/reports/' not in text


def test_quick_start_uses_typed_recording_directory():
    text = _guide()

    assert 'rosbags/semantic_search/recordings/' in text
    assert 'rosbags/semantic_search/raw/' not in text
    assert 'rosbags/semantic_search/bags/' not in text
