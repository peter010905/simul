"""Configure a generated model without modifying installed ROS descriptions."""

def configure_sensors(doc, camera_width=640, control_params=None, fuse_imu=False):
    import os
    width=int(os.environ.get('CAMERA_BENCH_WIDTH',camera_width))
    height=int(os.environ.get('CAMERA_BENCH_HEIGHT',width*3//4))
    fps=float(os.environ.get('CAMERA_BENCH_FPS','5'))
    if width not in (250,320,400,480,640) or height!=width or fps not in (5.,10.,15.):
        raise ValueError('Expected a square 250/320/400/480/640 camera at 5/10/15 FPS')
    for element in doc.getElementsByTagName('render_engine'):
        if element.firstChild:element.firstChild.data='ogre2'
    if control_params:
        for element in doc.getElementsByTagName('parameters'):
            if element.firstChild and 'irobot_create_control' in element.firstChild.data:
                element.firstChild.data=str(control_params)
    if fuse_imu:
        for element in doc.getElementsByTagName('remapping'):
            if element.firstChild and element.firstChild.data=='~/odom:=odom':
                element.firstChild.data='~/odom:=wheel/odom'
    for sensor in doc.getElementsByTagName('sensor'):
        rgbd=sensor.getAttribute('type')=='rgbd_camera'
        for rate in sensor.getElementsByTagName('update_rate'):
            if rate.firstChild:rate.firstChild.data=str(fps if rgbd else min(float(rate.firstChild.data),100 if sensor.getAttribute('type')=='imu' else 10))
        if rgbd:
            for img in sensor.getElementsByTagName('image'):
                for tag,value in [('width',width),('height',height)]:
                    img.getElementsByTagName(tag)[0].firstChild.data=str(value)
    return doc
