import json
import os
from pathlib import Path
from tempfile import TemporaryDirectory
import threading
import time
from types import SimpleNamespace
import unittest

from experiment_library.catalog import FOCUS, SURVEY, Library, version
from experiment_library.install import install, LEGACY, MEDIA


class LibraryTests(unittest.TestCase):
    def setUp(self):
        temp=TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        base=Path(temp.name)
        self.args=SimpleNamespace(cards=base/'cards',source=base/'archive',track_source=base/'track')
        self.args.cards.mkdir()
        self.board=SimpleNamespace(scanned=time.time(),lock=threading.Lock(),runs={})

    def run_card(self, ident, phase, kind='spread_exam_boxes', boxes=3, age=0, generator=None):
        name='v151-survey-candidate-track-'+ident
        card=self.args.cards/name
        card.mkdir()
        raw=self.args.cards.parent/'recordings'/name
        raw.mkdir(parents=True)
        (raw/'policy_revision.txt').write_text('v152_focus_continuity_box1122_ws')
        scene=dict(seed=123,randomization=dict(furniture='fixed',obstacles=kind,generator=generator),
                   obstacles=[dict(shape='box')]*boxes,targets=[{}, {}, {}])
        (raw/'scene.json').write_text(json.dumps(scene))
        trajectory=raw/'trajectory.csv'
        trajectory.write_text('data\n')
        when=time.time()-age
        os.utime(trajectory,(when,when))
        os.utime(raw/'policy_revision.txt',(when,when))
        if phase=='finished':
            (card/'simulation_evaluation.json').write_text('{"exam_score":{"verified":2,"registered":3,"elapsed_s":72.65,"strict":false}}')
        def summary(now):
            return dict(id=ident,phase=phase,sim_time=72,confirmed=[],approached=[],bumper_events=0)
        self.board.runs[ident]=SimpleNamespace(id=ident,label=name,path=card,summary=summary)
        return trajectory

    def test_identity_comes_from_controller_even_when_run_names_match(self):
        name='v151-survey-candidate-track-abc'
        self.assertEqual(version(name,'','dfd33c79d93498337a7f4257900f724894e7b5048f2dfee67a39ab395b3691bd'),FOCUS)
        self.assertEqual(version(name,'','46a6dd68c5070172a22b6680f2945cf22aa8b4f3866797316c820e9fb82e135a'),SURVEY)
        self.assertNotEqual(version(name,'v152_focus_continuity_box1122_ws','1234abcd'),FOCUS)

    def test_history_visibility_survives_restart_and_does_not_modify_recordings(self):
        path=self.run_card('new','finished')
        self.run_card('active','running')
        self.run_card('stale','running',age=600)
        self.run_card('empty','waiting',age=3600)
        self.run_card('legacy','finished','legacy_mixed',4)
        self.run_card('five','finished','exam_boxes',5)
        self.run_card('random_zero','finished','random_boxes',0,generator='exam-random-layout')
        self.run_card('random_four','finished','random_boxes',4,generator='exam-random-layout')
        lib=Library(self.board,self.args)
        current=lib.list()
        self.assertEqual(set(current['live']),{'active'})
        self.assertEqual({r['id'] for r in current['runs']},{'active','new','five','random_zero','random_four'})
        self.assertEqual({r['id']:r['phase'] for r in lib.list(True)['runs']},
                         dict(stale='stalled',empty='unstarted',legacy='finished'))
        row=next(r for r in current['runs'] if r['id']=='new')
        self.assertEqual((row['version'],row['boxes'],row['confirmed'],row['approached']),(FOCUS,3,3,2))
        self.assertEqual(row['sim_time'],72.65)
        with self.assertRaisesRegex(ValueError,'진행 중'):lib.set_archived('active',True)
        with self.assertRaises(KeyError):lib.set_archived('unknown',True)
        lib.set_archived('new',True)
        reopened=Library(self.board,self.args)
        self.assertIn('new',{r['id'] for r in reopened.list(True)['runs']})
        reopened.set_archived('new',False)
        self.assertEqual(json.loads(reopened.state.read_text()),{})
        self.assertEqual(path.read_text(),'data\n')

    def test_install_is_idempotent_and_source_guarded(self):
        launcher=self.args.cards/'board.py'
        original="def main():\n    if True:\n"+MEDIA+"\n"+LEGACY+"\n"
        launcher.write_text(original)
        first=install(launcher)
        installed=launcher.read_bytes()
        self.assertEqual(first,install(launcher))
        self.assertEqual(installed,launcher.read_bytes())
        self.assertEqual(launcher.with_name(launcher.name+'.before-version-library').read_text(),original)
        launcher.write_text('def main(): pass')
        with self.assertRaises(ValueError):install(launcher)
