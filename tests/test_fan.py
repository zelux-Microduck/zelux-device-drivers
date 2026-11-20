"""Service-only fan driver regression tests; no hardware is accessed."""
import unittest
import asyncio
from unittest.mock import AsyncMock
from zelux_drivers.radxa.fan import RadxaFan, FanRequestError, FanUnavailable, FanHelper, FanOutcomeUnknown
from zelux_drivers.contracts import FanDriver


class FanTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.fan = RadxaFan()
        self.instance = {"id": "fan-mwnrt89v-1", "name": "Cooling", "enabled": True,
                         "pwm": {"chip": 0, "channel": 0}, "period_ns": 40000,
                         "driver_id": "zelux.fan.radxa-pwm", "tach_gpio": None,
                         "pulses_per_rev": 2}
        self.status = {"api_version": 1, "ok": True, "mode": "auto",
                       "items": [{"id": self.instance["id"], "pct": 70}],
                       "inventory": {"instances": [self.instance]}}
        self.fan.helper.call = AsyncMock(return_value=self.status)

    async def test_manual_uses_stable_instance_id(self):
        await self.fan.refresh()
        self.fan.helper.call.reset_mock()
        self.fan.helper.call.return_value = {"ok": True, "pct": 45}
        result = await self.fan.set_pct(self.instance["id"], 45)
        self.assertTrue(result["ok"])
        self.fan.helper.call.assert_awaited_once_with(
            {"op": "set", "instance_id": self.instance["id"], "pct": 45})

    async def test_legacy_index_resolves_from_inventory(self):
        await self.fan.refresh()
        await self.fan.set_pct(0, 50)
        self.assertEqual(self.fan.helper.call.call_args.args[0]["instance_id"], self.instance["id"])

    async def test_snapshot_preserves_inventory_and_live_items(self):
        await self.fan.refresh()
        snapshot = self.fan.snapshot_fan()
        self.assertEqual(snapshot["inventory"]["instances"], [self.instance])
        self.assertEqual(snapshot["items"], self.status["items"])

    async def test_fresh_driver_has_no_default_fans(self):
        self.assertEqual(self.fan.fans, [])
        self.assertEqual(self.fan.snapshot_fan()["items"], [])
        self.assertIsInstance(self.fan, FanDriver)

    async def test_create_refreshes_inventory(self):
        await self.fan.create_instance(self.instance)
        self.assertEqual(self.fan.helper.call.call_args_list[0].args[0],
                         {"op": "create", "instance": self.instance})
        self.assertEqual(self.fan.snapshot_fan()["inventory"]["instances"], [self.instance])

    async def test_lifecycle_uses_service_only(self):
        await self.fan.update_instance(self.instance["id"], self.instance)
        await self.fan.set_enabled(self.instance["id"], False)
        await self.fan.delete_instance(self.instance["id"])
        payloads = [call.args[0] for call in self.fan.helper.call.call_args_list]
        self.assertEqual([p["op"] for p in payloads],
                         ["update", "status", "disable", "status", "delete", "status"])
        self.assertTrue(payloads[-2]["confirm"])

    async def test_service_rejection_propagates(self):
        self.fan.helper.call.side_effect = FanRequestError(
            {"ok": False, "code": "resource_busy", "error": "occupied"})
        with self.assertRaises(FanRequestError):
            await self.fan.create_instance(self.instance)

    async def test_committed_create_survives_refresh_failure(self):
        self.fan.helper.call.side_effect = [
            {"ok": True, "inventory": {"instances": [self.instance]}},
            FanUnavailable("read timed out")]
        result = await self.fan.create_instance(self.instance)
        self.assertTrue(result["ok"])
        self.assertTrue(result["status_stale"])
        self.assertIn("refresh_warning", result)
        self.assertEqual(self.fan.snapshot_fan()["inventory"]["instances"], [self.instance])
        self.assertEqual(sum(c.args[0]["op"] == "create" for c in self.fan.helper.call.call_args_list), 1)

    async def test_committed_delete_does_not_resurrect_cached_instance(self):
        await self.fan.refresh()
        self.fan.helper.call.side_effect = [
            {"ok": True, "inventory": {"instances": []}}, FanUnavailable("offline")]
        result = await self.fan.delete_instance(self.instance["id"])
        self.assertTrue(result["ok"])
        self.assertEqual(self.fan.snapshot_fan()["items"], [])
        self.assertEqual(self.fan.snapshot_fan()["inventory"]["instances"], [])

    async def test_start_when_service_down_keeps_recovery_task(self):
        self.fan.helper.call.side_effect = FanUnavailable("down")
        await self.fan.start()
        try:
            self.assertFalse(self.fan.available)
            self.assertFalse(self.fan._task.done())
            self.fan.helper.call.side_effect = None
            await self.fan.refresh()
            self.assertTrue(self.fan.available)
        finally:
            await self.fan.stop()
        self.assertIsNone(self.fan._task)

    async def test_actual_helper_write_timeout_has_unknown_outcome(self):
        from unittest.mock import patch, Mock
        reader, writer = AsyncMock(), AsyncMock()
        reader.readline.side_effect = asyncio.TimeoutError()
        writer.write, writer.close = Mock(), Mock()
        with patch("asyncio.open_unix_connection", AsyncMock(return_value=(reader, writer)), create=True):
            with self.assertRaises(FanOutcomeUnknown):
                await FanHelper("/fake").call({"op": "create", "instance": self.instance})
        writer.write.assert_called_once()

    async def test_read_timeout_is_not_a_failed_write(self):
        from unittest.mock import patch, Mock
        reader, writer = AsyncMock(), AsyncMock()
        reader.readline.side_effect = asyncio.TimeoutError()
        writer.write, writer.close = Mock(), Mock()
        with patch("asyncio.open_unix_connection", AsyncMock(return_value=(reader, writer)), create=True):
            with self.assertRaises(FanUnavailable) as caught:
                await FanHelper("/fake").call({"op": "status"})
        self.assertNotIsInstance(caught.exception, FanOutcomeUnknown)


if __name__ == "__main__":
    unittest.main()
