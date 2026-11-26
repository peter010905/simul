"""Allow brief surface looks on exploration transit, not marker approaches."""


def inspectable_route(selected):
    return bool(selected) and not any(
        key in selected for key in ('approach_id', 'focus_id', 'print_point')
    )
