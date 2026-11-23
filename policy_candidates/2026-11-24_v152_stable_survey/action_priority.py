"""Keep Nav2 cancellation ownership with the strongest current reason."""

CONTACT_REASONS = {'bump_contact', 'scan_stale_during_undock', 'edge_print_hazard'}
SENSOR_REASONS = {
    'processed_camera_stale', 'scan_stale', 'map_unavailable',
    'map_to_base_tf_unavailable', 'nav2_unavailable', 'drive_unavailable',
}
PROGRESS_REASONS = {'navigation_stalled', 'action_timeout', 'undock_timeout'}


def priority(reason):
    if reason in CONTACT_REASONS:
        return 4
    if reason in SENSOR_REASONS:
        return 3
    if reason in PROGRESS_REASONS:
        return 2
    return int(bool(reason))
