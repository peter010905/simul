import unittest
from box_batch import BoxBatch


class BoxBatchTests(unittest.TestCase):
    def setUp(self):
        self.batch = BoxBatch()
        self.batch.update([(0.,0.), (4.,0.)], [], (0.,1.))
        self.batch.dispatched({'batch_box': (0.,0.)})

    def goal(self, x, distance=2.5):
        return (distance, dict(surface_odom=(x,.3), previous_attempts=0))

    def test_unseen_face_on_current_box_precedes_shorter_other_trip(self):
        self.assertLess(self.batch.rank(self.goal(0)),self.batch.rank(self.goal(4,2.)))

    def test_verified_box_deprioritized_but_retained(self):
        self.batch.update([(0.,0.), (4.,0.)],[(0.,.35)], (0.,1.))
        self.assertLess(self.batch.rank(self.goal(4)),self.batch.rank(self.goal(0)))

    def test_ambiguous_association_does_not_mark_either_box(self):
        self.batch.update([(0.,0.), (1.,0.)],[(.5,0.)], (0.,1.))
        self.assertFalse(self.batch.completed)

    def test_local_preference_expires_after_leaving_or_long_detour(self):
        self.assertTrue(self.batch.rank(self.goal(0,4.))[1])
        self.batch.update([(0.,0.)],[],(4.,0.))
        self.assertTrue(self.batch.rank(self.goal(0))[1])


if __name__ == '__main__':
    unittest.main()
