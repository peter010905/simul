"""Regression tests for explicit start, stale sensors, duplicate and lost RPCs."""
import json
import math
from pathlib import Path
import subprocess
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import robot_session as session


class StartSafetyTest(unittest.TestCase):
    def healthy(self):
        sensors = session.Sensors()
        for name in session.REQUIRED:
            value = [] if name == "hazards" else False if name == "dock" else (0., 0.) if name == "odom" else None
            sensors.update(name, 10., 100. if name in ("scan", "camera", "camera_info", "odom") else None, value)
        return sensors

    def test_real_sensor_clock_mismatch_blocks_start(self):
        sensors = self.healthy()
        self.assertEqual(sensors.reason(10.1, 100.1), "")
        self.assertIn("clock_or_stamp_stale", sensors.reason(10.1, 103.))
        self.assertIn("clock_or_stamp_stale", sensors.reason(10.1, 99.))
        self.assertIn("not_receiving", sensors.reason(12.1, 100.1))

    def test_real_hazards_and_dock_reject_but_backup_limit_does_not(self):
        sensors = self.healthy()
        sensors.update("hazards", 10., value=[0, 5])
        self.assertEqual(sensors.reason(10.1, 100.1), "")
        for hazard in (1, 2, 3, 4):
            sensors.update("hazards", 10., value=[hazard])
            self.assertIn("active_hazard", sensors.reason(10.1, 100.1))
        sensors.update("hazards", 10., value=[])
        sensors.update("dock", 10., value=True)
        self.assertIn("robot_docked", sensors.reason(10.1, 100.1))

    def test_stop_needs_recent_finite_measured_odometry(self):
        sensors = session.Sensors()
        self.assertFalse(sensors.stationary(10.))
        for now in (9.7, 9.8, 9.9):
            sensors.update("odom", now, value=(0., 0.))
        self.assertTrue(sensors.stationary(10.))
        self.assertFalse(sensors.stationary(12.))
        for value in ((0.1, 0.), (0., 0.1), (math.nan, 0.)):
            sensors.update("odom", 10., value=value)
            self.assertFalse(sensors.stationary(10.))

    def test_fresh_dock_reports_even_when_lidar_sleeps(self):
        sensors = session.Sensors()
        sensors.update("dock", 10., value=True)
        self.assertIn("robot_docked", sensors.reason(10.1, 100.))
        self.assertEqual(sensors.reason(13., 103.), "waiting_for_scan")

    def test_camera_is_rejected_before_mission_would_reject_it(self):
        sensors = self.healthy()
        sensors.update("camera", 10., 99.2)
        self.assertEqual(sensors.reason(10., 100.), "camera_clock_or_stamp_stale")

    def test_camera_repair_refuses_active_navigation(self):
        io = Mock()
        io.conflicts.return_value = ["mission"]
        with self.assertRaises(RuntimeError):
            session.repair_camera(io, session.StopFlag(), Mock())
        io.node.create_client.assert_not_called()

    def test_single_bump_allows_existing_mission_recovery(self):
        guard = session.BumpRecoveryGuard()
        self.assertEqual(guard.check(0., [1]), "")
        self.assertEqual(guard.check(1., [1]), "")
        self.assertEqual(guard.total, 1)
        self.assertEqual(guard.check(3., []), "")

    def test_held_bump_stops_if_retreat_does_not_clear_it(self):
        guard = session.BumpRecoveryGuard()
        guard.check(0., [1])
        self.assertIn("held", guard.check(10., [1]))

    def test_any_number_of_released_contacts_allows_recovery(self):
        guard = session.BumpRecoveryGuard()
        for index in range(100):
            now = index * 3.
            self.assertEqual(guard.check(now, [1]), "")
            self.assertEqual(guard.check(now+1., []), "")
        self.assertEqual(guard.total, 100)

    def test_cliff_stall_drop_always_stop_even_with_bump(self):
        for hazard in (2, 3, 4):
            self.assertIn("Hardware", session.BumpRecoveryGuard().check(0., [1, hazard]))

    def client(self):
        client = Mock()
        client.service_is_ready.return_value = True
        client.call_async.return_value.done.return_value = False
        return client

    def test_start_is_immediate_when_ready_without_fixed_dwell(self):
        gate, client = session.StartGate(), self.client()
        self.assertFalse(gate.step(10., False, "", "idle", client, object))
        client.call_async.assert_called_once()
        client.call_async.return_value.done.return_value = True
        client.call_async.return_value.result.return_value = SimpleNamespace(success=True)
        self.assertTrue(gate.step(10.01, False, "", "running", client, object))

    def test_stopped_and_stale_inputs_never_send_start(self):
        gate, client = session.StartGate(), self.client()
        with self.assertRaises(InterruptedError):
            gate.step(10., True, "", "idle", client, object)
        gate.step(10., False, "camera_stale", "idle", client, object)
        client.call_async.assert_not_called()

    def test_existing_mission_never_receives_another_start(self):
        gate, client = session.StartGate(), self.client()
        with self.assertRaises(RuntimeError):
            gate.step(10., False, "", "running", client, object)
        client.call_async.assert_not_called()

    def test_transient_camera_delay_does_not_discard_start_response(self):
        gate, client = session.StartGate(), self.client()
        gate.step(10., False, "", "idle", client, object)
        client.call_async.return_value.done.return_value = True
        client.call_async.return_value.result.return_value = SimpleNamespace(success=True)
        self.assertTrue(gate.step(10.05, False, "camera_clock_or_stamp_stale", "running", client, object))
        client.call_async.assert_called_once()

    def test_lost_start_reply_is_not_retried_and_pending_is_removed(self):
        gate, client = session.StartGate(), self.client()
        gate.step(10., False, "", "idle", client, object)
        gate.step(11., False, "", "running", client, object)
        with self.assertRaises(TimeoutError):
            gate.step(15.1, False, "", "running", client, object)
        gate.cancel(client)
        client.call_async.assert_called_once()
        client.remove_pending_request.assert_called_once_with(client.call_async.return_value)
        client.call_async.return_value.cancel.assert_called_once()

    def test_readiness_rejection_only_retries_after_answer(self):
        gate, client = session.StartGate(), self.client()
        gate.step(10., False, "", "idle", client, object)
        client.call_async.return_value.done.return_value = True
        client.call_async.return_value.result.return_value = SimpleNamespace(success=False, message="Waiting for TF")
        gate.step(10.01, False, "", "idle", client, object)
        self.assertEqual(client.call_async.call_count, 1)
        gate.step(10.27, False, "", "idle", client, object)
        self.assertEqual(client.call_async.call_count, 2)

    def test_hazard_while_start_reply_pending_aborts(self):
        gate, client = session.StartGate(), self.client()
        gate.step(10., False, "", "idle", client, object)
        with self.assertRaises(RuntimeError):
            gate.step(10.1, False, "active_hazard_types:1", "idle", client, object)
        self.assertEqual(client.call_async.call_count, 1)

    def test_clock_checks_never_mutate_system_clock(self):
        with patch.object(session.subprocess, "run", side_effect=[
                SimpleNamespace(stdout="yes\n"), SimpleNamespace(stdout="192.168.185.3\n")]) as run:
            session.clock_check("192.168.185.3")
        self.assertTrue(all(call.args[0][1] in ("show", "show-timesync") for call in run.call_args_list))
        for replies in ([SimpleNamespace(stdout="no\n")],
                        [SimpleNamespace(stdout="yes\n"), SimpleNamespace(stdout="wrong\n")]):
            with patch.object(session.subprocess, "run", side_effect=replies), self.assertRaises(RuntimeError):
                session.clock_check("192.168.185.3")

    def test_preflight_never_starts_existing_navigation(self):
        io = Mock()
        io.safety_reason.return_value = ""
        io.conflicts.return_value = ["mission"]
        with self.assertRaisesRegex(RuntimeError, "Existing navigation"):
            session.preflight(io, session.StopFlag(), 1.)
        io.start_client.call_async.assert_not_called()
        io.zero.assert_not_called()

    def test_end_launch_escalates_after_wait_and_targets_group(self):
        process = Mock(pid=1234)
        process.poll.return_value = None
        process.wait.side_effect = [subprocess.TimeoutExpired("launch", 8), None]
        with patch.object(session.os, "killpg") as kill, patch.object(session, "group_active", return_value=False):
            session.end_launch(process)
        self.assertEqual([v.args[1] for v in kill.call_args_list], [session.signal.SIGINT, session.signal.SIGTERM])

    def test_dry_run_uses_prepared_flow_v4_and_never_builds(self):
        root = Path(__file__).resolve().parents[1]
        result = subprocess.run(["bash", str(root / "robot.sh"), "dry-run"], capture_output=True, text=True, check=True)
        self.assertIn("robot_session.py start", result.stdout)
        self.assertIn("face_intent_flow_v4r2_ws", result.stdout)
        self.assertNotIn("colcon", result.stdout)
        candidate = subprocess.run(["bash", str(root / "robot.sh"), "dry-run-v3-mppi"],
                                   capture_output=True, text=True, check=True)
        self.assertIn("face_intent_v3_warp_ws", candidate.stdout)
        self.assertIn("2026-11-13_face_intent_v2/policy.json", candidate.stdout)
        result = subprocess.run(["python3", str(root / "scripts/robot_session.py"), "dry-run"], capture_output=True, text=True, check=True)
        command = json.loads(result.stdout)
        self.assertIn("undock:=false", command)
        self.assertIn("use_sim_time:=false", command)


if __name__ == "__main__":
    unittest.main()
