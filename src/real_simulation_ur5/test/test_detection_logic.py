"""
Tests for sim_detection_node internals.
No ROS spin needed — we test the pure helper function _make_det directly.
"""
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).parents[1]))

from real_simulation_ur5.sim_detection_node import _make_det


def test_make_det_weed_side_fields():
    det = _make_det(cls_id=1, conf=0.85, cx=320, cy=240, w=80, h=60)
    assert det.bbox.center.position.x == 320.0
    assert det.bbox.center.position.y == 240.0
    assert det.bbox.size_x == 80.0
    assert det.bbox.size_y == 60.0
    assert len(det.results) == 1
    assert det.results[0].hypothesis.class_id == '1'
    assert abs(det.results[0].hypothesis.score - 0.85) < 1e-6


def test_make_det_weed_top_class_string():
    det = _make_det(cls_id=2, conf=0.72, cx=100, cy=100, w=50, h=50)
    assert det.results[0].hypothesis.class_id == '2'


def test_make_det_watermelon_class():
    det = _make_det(cls_id=0, conf=0.91, cx=200, cy=150, w=120, h=90)
    assert det.results[0].hypothesis.class_id == '0'
    assert det.bbox.size_x == 120.0


def test_make_det_confidence_range():
    for conf in [0.0, 0.5, 1.0]:
        det = _make_det(1, conf, 0, 0, 10, 10)
        assert det.results[0].hypothesis.score == conf


def test_make_det_zero_size():
    det = _make_det(1, 0.8, 320, 240, 0, 0)
    assert det.bbox.size_x == 0.0
    assert det.bbox.size_y == 0.0


def test_make_det_class_id_is_always_string():
    for cls_id in [0, 1, 2, 99]:
        det = _make_det(cls_id, 0.8, 0, 0, 10, 10)
        assert isinstance(det.results[0].hypothesis.class_id, str)


def test_make_det_single_hypothesis_per_detection():
    det = _make_det(1, 0.9, 100, 100, 50, 50)
    assert len(det.results) == 1


def test_make_det_bbox_center_matches_cx_cy():
    cx, cy = 123, 456
    det = _make_det(2, 0.8, cx, cy, 40, 40)
    assert det.bbox.center.position.x == float(cx)
    assert det.bbox.center.position.y == float(cy)
