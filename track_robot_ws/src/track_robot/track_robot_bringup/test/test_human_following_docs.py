"""Contracts for the supervised live human-following operator guide."""

from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[4]
GUIDE = (
    WORKSPACE_ROOT / 'docs' / 'guides' / 'human-following'
    / 'live-supervised-test.md'
)
REPORT_TEMPLATE = (
    WORKSPACE_ROOT / 'docs' / 'guides' / 'human-following'
    / 'gate-report-template.md'
)
PERCEPTION_DOC = (
    WORKSPACE_ROOT / 'src' / 'track_robot_perception'
    / 'docs' / 'human_tracking_reinforcement.md'
)
DECISION_DOC = (
    WORKSPACE_ROOT / 'src' / 'track_robot' / 'track_robot_decision'
    / 'docs' / 'outdoor_decision.md'
)
SAFETY_DOC = (
    WORKSPACE_ROOT / 'src' / 'track_robot' / 'track_robot_safety'
    / 'docs' / 'obstacle_safety.md'
)


def _text(path):
    return path.read_text(encoding='utf-8')


def _normalized(path):
    return ' '.join(_text(path).split()).lower()


def _start_command(mode):
    command = 'human_following_ctl start ' + '\\'
    command += '\n  --runtime-mode {} --hardware auto'.format(mode)
    if mode == 'active':
        command += ' --confirm-motion'
    return command


def test_guide_contains_exact_one_command_operations():
    text = _text(GUIDE)

    for command in (
            _start_command('shadow'),
            _start_command('active'),
            'human_following_ctl status',
            'human_following_ctl stop',
            "ros2 service call /safety/emergency_stop "
            "std_srvs/srv/Trigger '{}'",
            'ros2 topic echo /human_following/session_state',
            'ros2 topic info /cmd_vel --verbose'):
        assert command in text


def test_guide_documents_gates_and_no_automatic_resume():
    text = _normalized(GUIDE)

    for gate in ('gate a', 'gate b', 'gate c', 'gate d'):
        assert gate in text
    assert 'new wave' in text
    assert 'does not automatically resume' in text
    assert 'tracks lifted' in text
    assert '0.05 m/s' in text
    assert 'foam' in text
    assert 'never use a person as a collision obstacle' in text


def test_guide_states_current_operating_limitations():
    text = _normalized(GUIDE)

    for limitation in (
            'low obstacle', 'drop-off', 'terrain', 'weather', 'lidar-only'):
        assert limitation in text
    assert 'hardware-ready' in text
    assert 'gates a-d' in text


def test_gate_report_template_records_required_evidence():
    text = _text(REPORT_TEMPLATE)

    for field in (
            'Date', 'Commit', 'Operator', 'Runtime mode', 'Effective limits',
            'Topic rates', 'State transitions', 'Observed stops', 'Failures',
            'PASS', 'FAIL'):
        assert field in text


def test_component_documents_define_non_overlapping_ownership():
    perception = _normalized(PERCEPTION_DOC)
    decision = _normalized(DECISION_DOC)
    safety = _normalized(SAFETY_DOC)

    assert 'trusted fused target state' in perception
    assert 'does not authorize robot motion' in perception
    assert 'target usability and session intent' in decision
    assert 'does not arm the base' in decision
    assert 'final motion authorization' in safety
    assert 'zero-command enforcement' in safety
