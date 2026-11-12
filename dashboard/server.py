"""Local, read-only native Gazebo observatory. Python stdlib + existing OpenCV.

Completed evaluator results override stale live status. Camera workers only
subscribe to ROS; no policy, scenario, frozen runtime, or training state is edited.
"""
import argparse
import atexit
from datetime import datetime, timezone
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
from pathlib import Path
import re
import shlex
import subprocess
import threading
import time
from urllib.parse import parse_qs, urlparse

HERE = Path(__file__).resolve().parent


def read_json(path, default=None):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return {} if default is None else default


def key(path):
    return hashlib.sha256(str(path).encode()).hexdigest()[:20]


def age(path):
    try:
        return max(0, time.time() - path.stat().st_mtime)
    except OSError:
        return None


def tail(path, limit=160, max_bytes=65536):
    try:
        with path.open('rb') as stream:
            size = stream.seek(0, 2)
            stream.seek(max(0, size - max_bytes))
            lines = stream.read(max_bytes).decode('utf-8', errors='replace').splitlines()
        if size > max_bytes:
            lines = lines[1:]
        return '\n'.join(lines[-limit:])
    except OSError:
        return ''


def evaluate(result):
    """Current rubric, never infer verified success from an old success flag."""
    audit = result.get('ground_truth_audit', {})
    verified = audit.get('available') is True
    passed = bool(verified and audit.get('correct_ids') and audit.get('correct_positions')
                  and audit.get('all_approached')
                  and audit.get('approached_within_1_5m') == 3
                  and float(audit.get('approach_radius_m', 99)) <= 1.5
                  and result.get('targets') == 3 and result.get('collisions') == 0
                  and 0 < result.get('elapsed_seconds', 9999) <= 600
                  and result.get('reason') == 'all_targets_found')
    return {'available': verified, 'passed': passed,
            'found_ids': audit.get('found_ids', []),
            'closest_approach_m': audit.get('closest_approach_m', {}),
            'position_errors_m': audit.get('position_errors_m', {}),
            'actual_approached': audit.get('approached_within_1_5m'),
            'correct_ids': audit.get('correct_ids'),
            'correct_positions': audit.get('correct_positions')}


def read_json_lines_last(path):
    for line in reversed(tail(path, 3).splitlines()):
        try:
            return json.loads(line)
        except ValueError:
            pass
    return {}


def byte_range(value, size):
    match = re.fullmatch(r'bytes=(\d*)-(\d*)', value)
    if not match or not size or not any(match.groups()):
        raise ValueError('Invalid range')
    first, last = match.groups()
    if not first:
        count = int(last)
        if count <= 0:
            raise ValueError('Invalid suffix')
        return max(0, size - count), size - 1
    start = int(first)
    end = min(size - 1, int(last)) if last else size - 1
    if start >= size or end < start:
        raise ValueError('Unsatisfiable range')
    return start, end


class Observatory:
    def __init__(self, root, cache, cameras=True):
        self.root = root.resolve()
        self.native = self.root / 'training_robust/native90'
        self.cache = cache
        self.cameras = cameras
        self.workers = {}
        self.stopped = threading.Event()
        self.lock = threading.RLock()
        self.snapshot = {'campaigns': [], 'jobs': [], 'videos': []}
        self.paths = {}
        self.video_paths = {}
        self.last_error = ''

    def contained(self, value):
        path = Path(value).resolve()
        if not path.is_relative_to(self.native.resolve()):
            raise ValueError('Path is outside native experiment storage')
        return path

    def services(self, names):
        names = sorted(n for n in set(names) if re.fullmatch(r'osp-[a-zA-Z0-9_.@-]+\.service', n))
        if not names:
            return {}
        try:
            output = subprocess.run(['systemctl', '--user', 'show', *names, '--no-pager',
                                     '-p', 'Id', '-p', 'ActiveState', '-p', 'SubState', '-p', 'MainPID'],
                                    capture_output=True, text=True, timeout=3).stdout
            states = {}
            for block in output.split('\n\n'):
                fields = dict(line.split('=', 1) for line in block.splitlines() if '=' in line)
                if 'Id' in fields:
                    states[fields['Id']] = fields
            return states
        except (OSError, subprocess.TimeoutExpired):
            return {}

    def refresh(self):
        campaigns, jobs, paths = [], [], {}
        states = []
        for statefile in sorted(self.native.glob('*/state.json')):
            state = read_json(statefile)
            if isinstance(state.get('jobs'), list):
                states.append((statefile, state))
        units = [j.get('unit', '') for _, s in states for j in s['jobs']]
        coordinator_units=[s['coordinator_unit'] for _,s in sorted(states,key=lambda row:row[0].stat().st_mtime,reverse=True)
                           if s.get('coordinator_unit')]
        units += coordinator_units
        units += ['osp-fresh-map-cem-v2.service', 'osp-spatial-cem-v1.service', 'osp-spatial-cem-v2.service']
        services = self.services(units)
        for statefile, state in states:
            campaign_id = statefile.parent.name
            campaign_jobs = []
            for job in state['jobs']:
                if not job.get('output'):
                    continue
                try:
                    output = self.contained(job['output'])
                except ValueError:
                    continue
                episodes = sorted(output.glob('episodes/*'))
                episode = episodes[-1] if episodes else None
                identifier = key(output)
                scenario = job.get('scenario', {})
                live = read_json(episode / 'live_status.json') if episode else {}
                result = read_json(episode / 'evaluation.json') if episode else {}
                invalidation = read_json(output / 'environment_invalidation.json')
                service = services.get(job.get('unit'), {})
                active = service.get('ActiveState') == 'active'
                if result and invalidation.get('policy_training_sample_valid') is False:
                    status = 'invalid_environment'
                elif result:
                    status = 'passed' if evaluate(result)['passed'] else 'failed'
                elif active:
                    status = 'running' if live else 'starting'
                elif episode:
                    status = 'stopped'
                else:
                    status = 'queued'
                row = {'id': identifier, 'campaign': campaign_id, 'number': job.get('number'),
                       'phase': job.get('phase'), 'generation': job.get('generation', 0),
                       'seed': scenario.get('seed'), 'obstacles': scenario.get('actual_obstacles'),
                       'mount': scenario.get('mount_profile'), 'domain': job.get('domain'),
                       'floor_sensor_gate': job.get('floor_sensor_gate'),
                       'runtime_configuration_gate': job.get('runtime_configuration_gate'),
                       'arena': scenario.get('arena', {'shape':'pdf_original','width_m':20.15,'height_m':11.35}),
                       'unit': job.get('unit'), 'service': service.get('ActiveState', 'unknown'),
                       'state': status, 'elapsed': result.get('elapsed_seconds', live.get('status', {}).get('elapsed_seconds', 0)),
                       'targets': result.get('targets', live.get('status', {}).get('targets', 0)),
                       'contacts': result.get('collisions', live.get('status', {}).get('collisions', 0)),
                       'status_age': age(episode / 'live_status.json') if episode else None,
                       'reason': result.get('reason', live.get('status', {}).get('readiness_reason', ''))}
                jobs.append(row)
                campaign_jobs.append(row)
                paths[identifier] = {'output': output, 'episode': episode, 'job': row,
                                     'live': live, 'result': result}
            completed = [j for j in campaign_jobs if j['state'] in ('passed', 'failed', 'invalid_environment')]
            campaigns.append({'id': campaign_id, 'status': state.get('status'),
                              'generation': state.get('generation', 0), 'phase': state.get('phase'),
                              'total': len(campaign_jobs), 'completed': len(completed),
                              'passed': sum(j['state'] == 'passed' for j in completed),
                              'active': sum(j['state'] in ('running', 'starting') for j in campaign_jobs),
                              'reserved_holdout_consumed': state.get('reserved_holdout_consumed'),
                              'mtime': statefile.stat().st_mtime})
        campaigns.sort(key=lambda c: (c['active'] > 0, c['mtime']), reverse=True)
        videos, video_paths = self.library(paths)
        snapshot = {'updated_at': datetime.now(timezone.utc).isoformat(), 'campaigns': campaigns,
                    'jobs': jobs, 'videos': videos, 'observer_error': self.last_error,
                    'coordinator': next((services[name] for name in [*coordinator_units,'osp-spatial-cem-v2.service', 'osp-spatial-cem-v1.service', 'osp-fresh-map-cem-v2.service']
                                         if services.get(name, {}).get('ActiveState') == 'active'), {}),
                    'rubric': '정확한 ID 3개 · 실제 접근 ≤1.5m · 좌표 오차 ≤0.5m · 접촉 0 · 시뮬레이션 600초 이내'}
        with self.lock:
            self.snapshot, self.paths, self.video_paths = snapshot, paths, video_paths
        if self.cameras:
            self.sync_cameras([j for j in jobs if j['state'] in ('running', 'starting')])

    def library(self, paths):
        candidates = {}
        index = read_json(self.root / 'repository_snapshot/videos.json')
        for entry in index.get('videos', []):
            try:
                path = self.contained(self.root / entry['local_path'])
            except (KeyError, ValueError):
                continue
            candidates[path] = entry
        for entry in paths.values():
            if entry['episode']:
                candidates.setdefault(entry['episode'] / 'video/mission.mp4', {})
        videos, registry = [], {}
        for video, old in candidates.items():
            recording = read_json(video.parent / 'recording.json')
            result = read_json(video.parent.parent / 'evaluation.json')
            if not video.is_file() or not recording.get('frames') or not result:
                continue
            identifier = key(video)
            audit = evaluate(result)
            environment_valid = read_json(video.parent.parent.parent.parent / 'environment_invalidation.json').get('policy_training_sample_valid')
            row = {'id': identifier, 'name': old.get('asset', str(video.relative_to(self.native)).replace('/episodes/', ' / ').replace('/video/mission.mp4', '')),
                   'seconds': result.get('elapsed_seconds'), 'targets': result.get('targets'),
                   'contacts': result.get('collisions'), 'passed': audit['passed'] and environment_valid is not False,
                   'environment_valid': environment_valid,
                   'audit_available': audit['available'], 'reason': result.get('reason'),
                   'bytes': video.stat().st_size, 'mtime': video.stat().st_mtime,
                   'release_url': old.get('url'), 'url': '/media/video?id=' + identifier,
                   'thumbnail': '/media/thumb?id=' + identifier}
            videos.append(row)
            registry[identifier] = video
        videos.sort(key=lambda v: v['mtime'], reverse=True)
        return videos, registry

    def sync_cameras(self, jobs):
        wanted = {j['id']: j for j in jobs if j.get('domain') is not None}
        # The training controller already caps native concurrency at two.
        for identifier in list(self.workers):
            worker = self.workers[identifier]
            if identifier not in wanted or worker['process'].poll() is not None:
                self.stop_worker(identifier)
        for identifier, job in list(wanted.items())[:2]:
            if identifier in self.workers:
                continue
            output = self.cache / identifier
            output.mkdir(parents=True, exist_ok=True)
            # Fresh directory per job prevents previous-domain frames being shown as live.
            for file in output.glob('*'):
                file.unlink(missing_ok=True)
            command = 'source /opt/ros/jazzy/setup.bash\nexec ' + shlex.join([
                '/usr/bin/python3', str(HERE / 'camera_observer.py'), '--domain', str(job['domain']), '--output', str(output)])
            log = (output / 'observer.log').open('ab')
            process = subprocess.Popen(['/bin/bash', '-c', command], stdout=log, stderr=log)
            self.workers[identifier] = {'process': process, 'log': log}

    def stop_worker(self, identifier):
        worker = self.workers.pop(identifier)
        process = worker['process']
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
        worker['log'].close()

    def close(self):
        self.stopped.set()
        for identifier in list(self.workers):
            self.stop_worker(identifier)

    def run(self):
        while not self.stopped.is_set():
            try:
                self.refresh()
                self.last_error = ''
            except Exception as exc:
                self.last_error = f'{type(exc).__name__}: {exc}'
            self.stopped.wait(3)

    def detail(self, identifier):
        with self.lock:
            entry = self.paths.get(identifier)
        if not entry:
            raise KeyError(identifier)
        episode, live, result = entry['episode'], entry['live'], entry['result']
        events = []
        if episode:
            for line in tail(episode / 'marker_events.jsonl', 100).splitlines():
                try:
                    events.append(json.loads(line))
                except ValueError:
                    pass
        status = live.get('status', {})
        audit = evaluate(result) if result else None
        invalidation = read_json(entry['output'] / 'environment_invalidation.json')
        if audit and invalidation.get('policy_training_sample_valid') is False:
            audit['raw_rubric_passed'] = audit['passed']
            audit['passed'] = False
        camera = read_json(self.cache / identifier / 'metadata.json')
        camera_age = max(0, time.time() - camera['received_unix']) if camera.get('received_unix') else None
        frame = self.frame_path(entry)
        return {**entry['job'], 'status': status, 'metrics': live.get('runtime_metrics', {}),
                'detector': live.get('detector', {}), 'events': events, 'audit': audit,
                'confirmed_ids': audit['found_ids'] if audit else live.get('detector', {}).get('resolved_ids', []),
                'approached_ids': status.get('approached_ids', []),
                'result': result, 'environment_invalidation': invalidation,
                'exploration_memory': read_json_lines_last(episode / 'exploration_memory.jsonl') if episode else {},
                'camera': {**camera, 'age': camera_age,
                    'live': entry['job']['state'] in ('running', 'starting') and camera_age is not None and camera_age < 10},
                'frame_age': age(frame) if frame else None,
                'video_id': key(episode / 'video/mission.mp4') if episode and result else None,
                'artifacts': [name for name, path in self.artifacts(entry).items() if path.is_file()]}

    def frame_path(self, entry):
        if not entry['episode']:
            return None
        for name in ('last_frame.png', 'latest_frame.png', 'first_frame.png') if entry['result'] else ('latest_frame.png', 'first_frame.png'):
            path = entry['episode'] / 'video' / name
            if path.is_file():
                return path
        return None

    def artifacts(self, entry):
        episode = entry['episode']
        return {} if not episode else {
            'floor_probe.json': entry['output'] / 'floor_probe.json',
            'runtime_configuration.json': entry['output'] / 'runtime_configuration.json',
            'runtime_configuration_check.json': entry['output'] / 'runtime_configuration_check.json',
            'evaluation.json': episode / 'evaluation.json',
            'marker_events.jsonl': episode / 'marker_events.jsonl',
            'runtime_trace.csv': episode / 'runtime_trace.csv',
            'exploration_memory.jsonl': episode / 'exploration_memory.jsonl',
            'launch.log': episode / 'launch.log'}


class Handler(BaseHTTPRequestHandler):
    server_version = 'OSPObservatory/1'

    def log_message(self, *args):
        pass  # Polling would otherwise fill the journal with identical access lines.

    def send_headers(self, status, content_type, length=None, extra=None):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Content-Security-Policy', "default-src 'self'; img-src 'self' data:; style-src 'self'; script-src 'self'; connect-src 'self'; media-src 'self'; frame-ancestors 'self'")
        if length is not None:
            self.send_header('Content-Length', str(length))
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()

    def payload(self, data, content_type='application/json; charset=utf-8', status=200, extra=None):
        body = json.dumps(data, ensure_ascii=False, allow_nan=False).encode() if content_type.startswith('application/json') else data
        self.send_headers(status, content_type, len(body), extra)
        if self.command != 'HEAD':
            self.wfile.write(body)

    def file(self, path, download=False):
        size = path.stat().st_size
        start, end, code = 0, size - 1, 200
        extra = {'Accept-Ranges': 'bytes'}
        if download:
            extra['Content-Disposition'] = f'attachment; filename="{path.name}"'
        if self.headers.get('Range'):
            try:
                start, end = byte_range(self.headers['Range'], size)
                code = 206
                extra['Content-Range'] = f'bytes {start}-{end}/{size}'
            except ValueError:
                return self.payload(b'', 'text/plain', 416, {'Content-Range': f'bytes */{size}'})
        self.send_headers(code, mimetypes.guess_type(path.name)[0] or 'application/octet-stream', max(0, end - start + 1), extra)
        if self.command == 'HEAD':
            return
        with path.open('rb') as stream:
            stream.seek(start)
            remaining = end - start + 1
            while remaining > 0:
                block = stream.read(min(262144, remaining))
                if not block:
                    break
                self.wfile.write(block)
                remaining -= len(block)

    def do_HEAD(self):
        self.do_GET()

    def do_GET(self):
        try:
            self.route()
        except (BrokenPipeError, ConnectionResetError):
            pass
        except (KeyError, FileNotFoundError, ValueError):
            self.payload({'error': '요청한 기록이 없거나 아직 준비되지 않았습니다.'}, status=404)
        except Exception as exc:
            self.payload({'error': type(exc).__name__}, status=500)

    def route(self):
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query)
        get = lambda name, default='': query.get(name, [default])[0]
        store = self.server.observatory
        if parsed.path in ('/', '/app.js', '/style.css'):
            return self.file(HERE / 'web' / {'/': 'index.html', '/app.js': 'app.js', '/style.css': 'style.css'}[parsed.path])
        if parsed.path == '/api/state':
            with store.lock:
                return self.payload(store.snapshot)
        if parsed.path == '/api/job':
            return self.payload(store.detail(get('id')))
        if parsed.path == '/api/log':
            with store.lock:
                entry = store.paths[get('id')]
            source = get('source', 'launch')
            options = {'launch': entry['episode'] / 'launch.log' if entry['episode'] else entry['output'] / 'service.log',
                       'benchmark': entry['output'] / 'service.log',
                       'events': entry['episode'] / 'marker_events.jsonl' if entry['episode'] else entry['output'] / 'missing'}
            return self.payload({'text': tail(options[source], min(500, max(1, int(get('lines', '160'))))), 'source': source})
        if parsed.path == '/api/artifact':
            with store.lock:
                entry = store.paths[get('id')]
            return self.file(store.artifacts(entry)[get('name')], download=True)
        if parsed.path in ('/media/video', '/media/thumb'):
            with store.lock:
                video = store.video_paths[get('id')]
            if parsed.path == '/media/video':
                return self.file(video, get('download') == '1')
            return self.file(video.parent / 'last_frame.png' if (video.parent / 'last_frame.png').exists() else video.parent / 'first_frame.png')
        if parsed.path == '/api/frame':
            import cv2
            with store.lock:
                entry = store.paths[get('id')]
            view = get('view', 'composite')
            if view not in ('robot', 'map', 'overview', 'composite'):
                raise ValueError(view)
            camera = store.cache / get('id') / 'frame.jpg'
            if view == 'robot' and entry['job']['state'] in ('running', 'starting') and (age(camera) or 999) < 10:
                return self.file(camera)
            path = store.frame_path(entry)
            if path is None:
                raise FileNotFoundError()
            image = cv2.imread(str(path))
            if image is None:
                raise FileNotFoundError()
            if view == 'robot':
                image = image[522:708, 12:322]
            elif view == 'map':
                image = image[44:670, 808:1268]
            elif view == 'overview':
                image = image[72:482, 12:796]
            ok, data = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                raise ValueError('Cannot decode frame')
            return self.payload(data.tobytes(), 'image/jpeg')
        raise KeyError(parsed.path)


def serve(root, port=8765, host='127.0.0.1', cameras=True, cache=None):
    cache = cache or Path('/tmp') / f'osp-dashboard-{__import__("os").getuid()}'
    cache.mkdir(parents=True, exist_ok=True, mode=0o700)
    store = Observatory(root, cache, cameras)
    store.refresh()
    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    server.observatory = store
    atexit.register(store.close)
    threading.Thread(target=store.run, daemon=True).start()
    print(f'Robot dashboard: http://{host}:{server.server_port}', flush=True)
    try:
        server.serve_forever()
    finally:
        store.close()
        server.server_close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, default=HERE.parent)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--no-cameras', action='store_true')
    args = parser.parse_args()
    serve(args.root, args.port, cameras=not args.no_cameras)
