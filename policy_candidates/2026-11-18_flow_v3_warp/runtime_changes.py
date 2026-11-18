def apply(package):
    path = package / 'osp_mission/marker_pair.py'
    text = path.read_text()
    changes = [
        ('import itertools', 'import itertools\nfrom osp_mission.warped_marker import decode_rejected'),
        ('corners, ids, _ = cv2.aruco.detectMarkers(frame, dictionary)', '''corners, ids, rejected = cv2.aruco.detectMarkers(frame, dictionary)
        if scale == 1:
            native_rejected = rejected'''),
        ('    if not results:', '''    for quad in native_rejected:
        if any(overlapping(quad, other) for other, _ in results):
            continue
        recovered = decode_rejected(image, quad)
        if recovered is not None:
            results.append(recovered)
    if not results:''')]
    for old, new in changes:
        if text.count(old) != 1:
            raise ValueError(old)
        text = text.replace(old, new)
    path.write_text(text)
