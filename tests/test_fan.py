"""测试风扇驱动实例管理方法的 socket payload。"""
from unittest.mock import AsyncMock, MagicMock

from zelux_drivers.radxa.fan import RadxaFan, FanHelper
from zelux_drivers.contracts import FanDriver


class FakeContext:
    class Config:
        fan_socket = "/run/zelux/fan.sock"
        fan_default_pct = 60
        fan_curve = []

    config = Config()

    def __init__(self):
        import logging
        self.log = logging.getLogger("test.fan")


def create_fan_with_mock_helper():
    """创建一个 RadxaFan 实例，注入 mock helper。"""
    helper = MagicMock(spec=FanHelper)
    helper.exists.return_value = True
    helper.call = AsyncMock(return_value={"ok": True})

    ctx = FakeContext()
    fan = RadxaFan(ctx, fans=({"name": "测试风扇", "pwm": 14, "gpio": 116, "tach": None},))
    fan.helper = helper
    fan.available = True
    fan.source = "helper"
    return fan, helper


class TestFanInstanceManagement:
    """测试风扇实例管理的 socket payload。"""

    def test_create_instance_payload(self):
        """create_instance 应该发送 {op:create, instance}。"""
        import asyncio
        fan, mock_helper = create_fan_with_mock_helper()
        instance = {"name": "新风扇", "pwm": 14, "curve": [[50, 60]]}

        result = asyncio.run(fan.create_instance(instance))

        assert result["ok"] is True
        mock_helper.call.assert_called_once()
        payload = mock_helper.call.call_args[0][0]
        assert payload["op"] == "create"
        assert payload["instance"] == instance

    def test_update_instance_payload(self):
        """update_instance 应该发送 {op:update, instance_id, instance}。"""
        import asyncio
        fan, mock_helper = create_fan_with_mock_helper()
        instance = {"name": "更新风扇", "pwm": 14, "curve": [[55, 65]]}

        result = asyncio.run(fan.update_instance("fan-1", instance))

        assert result["ok"] is True
        mock_helper.call.assert_called_once()
        payload = mock_helper.call.call_args[0][0]
        assert payload["op"] == "update"
        assert payload["instance_id"] == "fan-1"
        assert payload["instance"] == instance

    def test_delete_instance_payload(self):
        """delete_instance 应该发送 {op:delete, instance_id, confirm:true}。"""
        import asyncio
        fan, mock_helper = create_fan_with_mock_helper()

        result = asyncio.run(fan.delete_instance("fan-1"))

        assert result["ok"] is True
        mock_helper.call.assert_called_once()
        payload = mock_helper.call.call_args[0][0]
        assert payload["op"] == "delete"
        assert payload["instance_id"] == "fan-1"
        assert payload["confirm"] is True

    def test_set_enabled_enable_payload(self):
        """set_enabled(True) 应该发送 {op:enable, instance_id}。"""
        import asyncio
        fan, mock_helper = create_fan_with_mock_helper()

        result = asyncio.run(fan.set_enabled("fan-1", True))

        assert result["ok"] is True
        mock_helper.call.assert_called_once()
        payload = mock_helper.call.call_args[0][0]
        assert payload["op"] == "enable"
        assert payload["instance_id"] == "fan-1"

    def test_set_enabled_disable_payload(self):
        """set_enabled(False) 应该发送 {op:disable, instance_id}。"""
        import asyncio
        fan, mock_helper = create_fan_with_mock_helper()

        result = asyncio.run(fan.set_enabled("fan-1", False))

        assert result["ok"] is True
        mock_helper.call.assert_called_once()
        payload = mock_helper.call.call_args[0][0]
        assert payload["op"] == "disable"
        assert payload["instance_id"] == "fan-1"

    def test_unavailable_returns_error(self):
        """当 available=False 时应该返回错误而不是调用 helper。"""
        import asyncio
        fan, mock_helper = create_fan_with_mock_helper()
        fan.available = False
        fan.reason = "测试不可用"
        fan.hint = "测试提示"

        result = asyncio.run(fan.create_instance({"name": "test"}))

        assert result["ok"] is False
        assert result["available"] is False
        assert "测试不可用" in result["reason"]
        mock_helper.call.assert_not_called()


class TestFanDriverContract:
    """验证 RadxaFan 满足 FanDriver 协议。"""

    def test_radxa_fan_satisfies_contract(self):
        """RadxaFan 应该实现 FanDriver 的所有方法。"""
        fan, _ = create_fan_with_mock_helper()
        assert isinstance(fan, FanDriver)

        # 检查所有方法都存在且是协程
        import inspect
        for method_name in ["create_instance", "update_instance", "delete_instance", "set_enabled"]:
            method = getattr(fan, method_name)
            assert callable(method)
            assert inspect.iscoroutinefunction(method)