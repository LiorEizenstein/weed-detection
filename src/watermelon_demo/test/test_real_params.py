"""test_real_params.py — validate real_params.yaml is well-formed and complete."""
import os
import pytest

try:
    import yaml
except ImportError:
    pytest.skip('pyyaml not available', allow_module_level=True)

REAL_PARAMS = os.path.join(
    os.path.dirname(__file__), '..', 'config', 'real_params.yaml')


def _load():
    with open(REAL_PARAMS) as f:
        return yaml.safe_load(f)


class TestRealParams:
    def test_file_exists(self):
        assert os.path.isfile(REAL_PARAMS), f"real_params.yaml not found at {REAL_PARAMS}"

    def test_arm_controller_has_dry_run(self):
        data = _load()
        params = data['arm_controller_node']['ros__parameters']
        assert 'dry_run' in params, "arm_controller_node missing 'dry_run'"
        assert isinstance(params['dry_run'], bool)

    def test_use_sim_time_false(self):
        data = _load()
        for node_name, node_data in data.items():
            p = node_data.get('ros__parameters', {})
            if 'use_sim_time' in p:
                assert p['use_sim_time'] is False, (
                    f"{node_name}: use_sim_time must be false for real hardware")

    def test_use_real_model_true(self):
        data = _load()
        p = data['detection_node']['ros__parameters']
        assert p['use_real_model'] is True, "detection_node must use real YOLO model"

    def test_save_debug_frames_false(self):
        data = _load()
        p = data['detection_node']['ros__parameters']
        assert p.get('save_debug_frames') is False, (
            "save_debug_frames should be false (no log dir guaranteed on robot)")
