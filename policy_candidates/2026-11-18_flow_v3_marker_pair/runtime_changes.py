"""Install the paired marker decoder and extend close-ID resolution."""
def apply(package):
    def replace(filename, changes):
        path = package / 'osp_mission' / filename
        text = path.read_text()
        for old, new in changes:
            if text.count(old) != 1:
                raise ValueError(f'Unexpected source in {filename}: {old}')
            text = text.replace(old, new)
        path.write_text(text)

    replace('landmark_identity.py', [
        ('confirmations=3, conflict_radius=.12)', 'confirmations=3, conflict_radius=.12, separate=None)'),
        ('if distance>conflict_radius:', 'if distance>conflict_radius or (separate and separate.established(a,b)):'),
    ])
    replace('detector.py', [
        ('from osp_mission.landmark_identity import', 'from osp_mission.marker_pair import SeparateMarkers, decode\nfrom osp_mission.landmark_identity import'),
        ('        self.dictionary=', '        self.separate_markers=SeparateMarkers()\n        self.dictionary='),
        ('corners,ids,_=cv2.aruco.detectMarkers(image,self.dictionary)', 'corners,ids=decode(image,self.dictionary)\n            accepted_quads={}'),
        ("                self.diagnostics['accepted']+=1", "                accepted_quads[ident]=c\n                self.diagnostics['accepted']+=1"),
        ('        except Exception as exc:\n            self.diagnostics', "            if accepted_quads:\n                self.separate_markers.observe(stamp.nanoseconds,accepted_quads)\n        except Exception as exc:\n            self.diagnostics"),
        ("resolved_ids(self.observations,self.p('confirmations'))","resolved_ids(self.observations,self.p('confirmations'),separate=getattr(self,'separate_markers',None))"),
    ])
