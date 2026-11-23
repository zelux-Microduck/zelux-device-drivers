"""Production servo HAL must proxy operations through robotd, never open the UART."""
import asyncio
import io
import json
import logging
import os
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from zelux_drivers import create


class ServoRobotdProxyTests(unittest.IsolatedAsyncioTestCase):
    def make_servo(self):
        with patch.dict(os.environ, {"ZELUX_SERVO_SIM": "0"}):
            servo = create("zelux.servo.feetech-sts-scs",
                           SimpleNamespace(config=SimpleNamespace(), log=logging.getLogger("test")))
        servo._session_id = "session-token"
        servo._taken = True
        return servo

    async def test_register_write_is_typed_robotd_command(self):
        servo = self.make_servo()
        calls = []
        servo._robotd_call_sync = lambda params: calls.append(params) or {"data": [111]}
        await servo.servo_write(20, 31, -2, eeprom=True, confirmed=True)
        self.assertEqual(calls[0]["action"], "command")
        self.assertEqual(calls[0]["session_id"], "session-token")
        self.assertEqual(calls[0]["command"], {
            "op": "write_register", "servo_id": 20, "address": 31,
"data": [2, 128], "eeprom": True, "confirmed": True,
        })

    async def test_unconfirmed_ram_write_is_forwarded_as_unconfirmed(self):
        servo = self.make_servo()
        calls = []
        servo._robotd_call_sync = lambda params: calls.append(params) or {"data": [111]}
        await servo.servo_write(20, 21, 12)
        self.assertFalse(calls[0]["command"]["confirmed"])

    async def test_calibration_direction_and_offset_confirm_robotd_writes(self):
        servo = self.make_servo()
        calls = []
        def rpc(params):
            calls.append(params)
            if params["command"]["op"] == "read_register":
                return {"data": [0]}
            return {"data": [111]}
        servo._robotd_call_sync = rpc
        await servo.set_direction(20, -1)
        await servo.save_offsets([(20, -2)])
        writes = [call["command"] for call in calls if call["command"]["op"] == "write_register"]
        self.assertEqual([(w["address"], w["eeprom"], w["confirmed"]) for w in writes],
                         [(18, True, True), (31, True, True)])

    async def test_torque_and_synchronized_goals_are_proxied(self):
        servo = self.make_servo()
        calls = []
        servo._robotd_call_sync = lambda params: calls.append(params) or {"data": [111]}
        await servo.servo_torque(False, [10, 20])
        await servo.servo_sync_goal({10: 2048})
        self.assertEqual(calls[0]["command"], {
            "op": "set_torque", "on": False, "servo_ids": [10, 20],
        })
        goal = calls[1]["command"]["goals"][0]
        self.assertEqual((goal["servo_id"], goal["position"]), (10, 2048))
        self.assertGreaterEqual(goal["speed"], 0)

    async def test_take_releases_lease_when_initial_snapshot_fails(self):
        servo = self.make_servo()
        servo._taken = False
        servo._session_id = None
        calls = []
        def rpc(params):
            calls.append(params)
            if params["action"] == "begin":
                return {"session_id": "lease-1", "bus_protocol": "feetech-sts"}
            if params["action"] == "command":
                raise RuntimeError("snapshot failed")
            return {"active": False}
        servo._robotd_call_sync = rpc
        with self.assertRaises(RuntimeError):
            await servo.take()
        self.assertEqual([c["action"] for c in calls], ["begin", "command", "end"])
        self.assertFalse(servo._taken)
        self.assertIsNone(servo._session_id)
        self.assertEqual(servo._cache["bus"], "shared")

    async def test_take_ends_unsupported_protocol_before_refusing(self):
        servo = self.make_servo()
        servo._taken = False
        servo._session_id = None
        calls = []
        def rpc(params):
            calls.append(params)
            return {"session_id": "lease-1", "bus_protocol": "dynamixel2"}
        servo._robotd_call_sync = rpc
        with self.assertRaisesRegex(Exception, "does not support dynamixel2"):
            await servo.take()
        self.assertEqual([c["action"] for c in calls], ["begin", "end"])
        self.assertFalse(servo._taken)
        self.assertIsNone(servo._session_id)

    async def test_production_transport_refuses_direct_uart(self):
        servo = self.make_servo()
        with self.assertRaises(Exception):
            servo._make_link()


    async def test_factory_take_snapshot_release_uses_socket_not_uart(self):
        servo = self.make_servo()
        servo._taken = False
        servo._session_id = None
        responses = [
            {"session_id": "lease-factory", "bus_protocol": "feetech-sts"},
            {"data": list(json.dumps({"items": [
                {"id": 20, "raw": [0, 8, 0, 0, 0, 0, 80, 35], "torque": True}
            ]}).encode())},
            {"active": False},
        ]
        from unittest.mock import Mock
        sock = Mock()
        reader = io.StringIO("".join(json.dumps({"jsonrpc": "2.0", "id": n + 1,
                                               "result": result}) + "\n"
                                    for n, result in enumerate(responses)))
        sock.makefile.return_value = reader
        # Use the actual RPC implementation, not the per-test stub.
        with patch("zelux_drivers.radxa.servo.socket.AF_UNIX", 1, create=True), \
             patch("zelux_drivers.radxa.servo.socket.socket", return_value=sock), \
             patch("zelux_drivers.radxa.servo.SerialLink", side_effect=AssertionError("UART opened")) as uart, \
             patch.object(servo, "_start_poll"):
            result = await servo.take(supported=True)
            self.assertEqual(result["online"], 1)
            self.assertEqual(servo.snapshot_servos()["items"][0]["pos"], 2048)
            await servo.release()
        uart.assert_not_called()
        requests = [json.loads(call.args[0]) for call in sock.sendall.call_args_list]
        self.assertEqual([r["method"] for r in requests], ["robot.servoSession"] * 3)
        self.assertEqual([r["params"]["action"] for r in requests], ["begin", "command", "end"])
        self.assertIs(requests[0]["params"]["supported"], True)
        self.assertEqual(requests[1]["params"]["command"], {"op": "snapshot"})
        self.assertFalse(servo._taken)
        sock.close.assert_called_once()

    async def test_missing_robotd_never_falls_back_to_uart(self):
        servo = self.make_servo()
        servo._taken = False
        servo._session_id = None
        from unittest.mock import Mock
        sock = Mock()
        sock.connect.side_effect = OSError("missing robotd socket")
        with patch("zelux_drivers.radxa.servo.socket.AF_UNIX", 1, create=True), \
             patch("zelux_drivers.radxa.servo.socket.socket", return_value=sock), \
             patch("zelux_drivers.radxa.servo.SerialLink", side_effect=AssertionError("UART opened")) as uart:
            with self.assertRaisesRegex(Exception, "robotd servo session unavailable"):
                await servo.take()
        uart.assert_not_called()
        self.assertFalse(servo._taken)


if __name__ == "__main__":
    unittest.main()
