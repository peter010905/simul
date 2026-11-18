def apply(package):
    def replace(file, old, new):
        path = package / 'osp_mission' / file
        text = path.read_text()
        if text.count(old) != 1:
            raise ValueError(f'Unexpected parent in {file}: {old}')
        path.write_text(text.replace(old, new))

    replace('mission.py', 'from osp_mission.wall_route import WallLook, penalize_routes',
            'from osp_mission.surface_sweep import SurfaceSweep\nfrom osp_mission.wall_route import penalize_routes')
    replace('mission.py', 'self.wall_look=WallLook()', 'self.wall_look=SurfaceSweep()')
    # Compute the extent once, after the existing center ranking finishes.
    replace('wall_route.py', 'best=(score,tuple(center))', 'best=(score,tuple(center),segment)')
    replace('wall_route.py', '        return best[1] if best else None',
            '        if best:\n            self.choose_extent(best[2], best[1], camera)\n        return best[1] if best else None')
    replace('wall_route.py', '    def check(self, node, now):',
            '    def choose_extent(self, segment, center, camera):\n        pass\n\n'
            '    def observation_angle(self, node, turn, camera):\n        return turn\n\n'
            '    def check(self, node, now):')
    replace('wall_route.py', '        return turn\n\n\ndef penalize_routes', '        return self.observation_angle(node, turn, cam)\n\n\ndef penalize_routes')
