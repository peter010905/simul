"""Selective release publishing must not upload unrelated completed recordings."""
import contextlib
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import cv2
import numpy as np
import publish_simulation_videos as publisher


class SelectivePublishingTest(unittest.TestCase):
    def test_only_requested_video_uploads_and_complete_index_is_retained(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);(root/'repository_snapshot').mkdir();(root/'scripts').mkdir()
            (root/'README.md').write_text('Simulation recordings\n')
            videos=[]
            for name in ['requested','other']:
                episode=root/'training_robust/native90'/name/'episode'
                video=episode/'video';video.mkdir(parents=True)
                file=video/'mission.mp4'
                writer=cv2.VideoWriter(str(file),cv2.VideoWriter_fourcc(*'mp4v'),5,(64,48))
                assert writer.isOpened()
                for _ in range(2):writer.write(np.zeros((48,64,3),np.uint8))
                writer.release();videos.append(file)
                (video/'recording.json').write_text(json.dumps({'frames':2,'duration_seconds':.4}))
                (episode/'evaluation.json').write_text(json.dumps({'success':False,'reason':'timeout','targets':1,'collisions':0}))
            (videos[0].parent.parent.parent/'environment_invalidation.json').write_text(json.dumps(
                dict(policy_training_sample_valid=False,reason='Invalid floor sensor input')))
            assets=[];uploaded=[]
            def command(args):
                endpoint=args[-1]
                if endpoint.endswith('/releases/tags/simulation-videos-2026-11'):
                    return json.dumps({'id':1,'html_url':'https://github.com/owner/repo/releases/tag/test'})
                if 'assets?' in endpoint:return json.dumps([assets])
                return json.dumps({'private':False})
            def upload(args,**kwargs):
                path=Path(args[4]);uploaded.append(path.name)
                assets.append({'name':path.name,'size':path.stat().st_size,
                    'digest':'sha256:'+hashlib.sha256(path.read_bytes()).hexdigest()})
            with patch.object(publisher,'__file__',str(root/'scripts/publish_simulation_videos.py')), \
                 patch.object(publisher,'command',side_effect=command), \
                 patch.object(publisher.subprocess,'run',side_effect=upload), \
                 patch('sys.argv',['publish','--upload','--only-upload',str(videos[0])]), \
                 contextlib.redirect_stdout(io.StringIO()):
                publisher.main()
            self.assertEqual(uploaded,['requested__episode.mp4'])
            report=json.loads((root/'repository_snapshot/videos.json').read_text())
            self.assertEqual(len(report['videos']),2)
            self.assertEqual(report['uploaded_verified'],1)
            self.assertFalse(report['repository_private'])
            invalid=next(v for v in report['videos'] if v['asset']=='requested__episode.mp4')
            self.assertFalse(invalid['policy_training_sample_valid'])
            self.assertFalse(invalid['original_evaluator_success'])
            self.assertIn('환경 오류',(root/'repository_snapshot/videos.html').read_text())
            self.assertNotIn('로그인해야',(root/'repository_snapshot/videos.html').read_text())


if __name__=='__main__':unittest.main()
