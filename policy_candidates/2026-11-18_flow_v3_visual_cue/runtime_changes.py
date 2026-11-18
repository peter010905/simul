def apply(package):
    def edit(name, changes):
        p = package / 'osp_mission' / name
        text = p.read_text()
        for old, new in changes:
            if text.count(old) != 1:
                raise ValueError(f'{name}: unexpected source {old}')
            text = text.replace(old, new)
        p.write_text(text)
    edit('marker_pair.py', [
        ('def decode(image, dictionary):','def decode(image, dictionary, visual_quads=None):'),
        ('            native_rejected = rejected','''            native_rejected = rejected
            if visual_quads is not None:
                visual_quads.extend(corners)
                visual_quads.extend(rejected)'''),
    ])
    edit('detector.py', [
        ('from osp_mission.marker_pair import', 'from osp_mission.visual_cue import VisualCue\nfrom osp_mission.marker_pair import'),
        ('        self.separate_markers=', '''        self.visual_cue = VisualCue()
        self.visual_pub = self.create_publisher(String, 'mission/visual_bearing', 10)
        self.separate_markers='''),
        ('            corners,ids=decode(image,self.dictionary)', '''            quads = []
            corners,ids=decode(image,self.dictionary,quads)
            readable = any(np.linalg.norm(q[0]-np.roll(q[0],1,axis=0),axis=1).min()>=self.p('min_marker_pixels') for q in corners)
            hint = self.visual_cue.update(image, quads, readable, Time.from_msg(msg.header.stamp).nanoseconds/1e9, k[0], k[2])
            if hint is not None:
                self.visual_pub.publish(String(data=json.dumps(hint)))'''),
    ])
    edit('mission.py', [
        ("        self.create_subscription(String,'mission/marker_bearing'", "        self.create_subscription(String,'mission/visual_bearing',self.on_visual_bearing,10)\n        self.create_subscription(String,'mission/marker_bearing'"),
        ('    def on_marker_bearing(self, m):', '''    def on_visual_bearing(self, m):
        cue = json.loads(m.data)
        age = self.get_clock().now().nanoseconds/1e9 - cue['stamp']
        guard = self.contact_inspection
        if self.state != 'running' or guard.phase or not -.1 <= age <= .5 or self.now() < self.spin_blocked_until:
            return
        if self.near_pass.protected_navigation or self.marker_look.pending or self.face_look.pending:
            return
        if (self.pending or self.goal_handle) and self.kind != 'nav':
            return
        if guard.bearing is not None and self.now()-guard.bearing[0] <= 2.:
            return
        guard.bearing = (self.now(),cue['angle'])
        self.get_logger().info(f"Inspecting unidentified printed square at {math.degrees(cue['angle']):.0f} deg")

    def on_marker_bearing(self, m):'''),
    ])
