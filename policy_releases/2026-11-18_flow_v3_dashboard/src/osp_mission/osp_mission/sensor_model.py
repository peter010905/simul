"""Configure a generated model without modifying installed ROS descriptions."""

def configure_sensors(doc, camera_width=640, control_params=None, fuse_imu=False):
    width=int(camera_width)
    if width not in (320,640,960):raise ValueError('camera_width must be 320, 640 or 960')
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
            if rate.firstChild:rate.firstChild.data=str(5 if rgbd else min(float(rate.firstChild.data),100 if sensor.getAttribute('type')=='imu' else 10))
        if rgbd:
            for img in sensor.getElementsByTagName('image'):
                for tag,value in [('width',width),('height',width*3//4)]:
                    img.getElementsByTagName(tag)[0].firstChild.data=str(value)
    return doc
