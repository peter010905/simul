"""Identify surface-directed transit without preempting marker approach."""
import math


def side_face_route(selected):
    if any(key in selected for key in ('approach_id', 'focus_id', 'print_point')):
        return False
    if 'box_point' in selected or selected.get('face_inspection'):
        return True
    features = selected.get('features')
    if features is None or not len(features):
        return False
    gain = float(features[0])
    return math.isfinite(gain) and gain >= .35
