# 风扇实例 ID 调速修复（2026-11-20）

## 原因

Studio 使用稳定实例 ID 下发手动调速，但实际加载的独立驱动仍保留固定双风扇列表和索引校验，导致已创建实例报“只有 2 颗”。Agent 内置适配器和独立驱动行为不一致。

## 本次修复

- 独立驱动改为 fanctl 服务客户端，不再直接访问 PWM/sysfs/GPIO，也不再生成默认双风扇。
- 字符串实例 ID 直接通过 set/get/selftest 请求传递；旧整数索引从服务实例清单解析。
- 创建、更新、删除和启停使用服务实例契约；状态从 items 和 inventory.instances 读取。
- 服务负责温控，Agent/驱动不再运行第二套硬件控制循环。

## 验证与部署范围

新增标准库 unittest 回归测试，覆盖真实格式实例 ID 调速、整数索引兼容、空清单、状态字段、生命周期和错误透传。需要在设备上更新实际 Python 环境中的 zelux-device-drivers 包并重启 zeluxd；仅更新 Studio 或 Agent 源码不会替换已安装的独立驱动包。不要重新创建已存在的实例。

## 后续收口结果（同日）

fanctl 1.1.1 工作区已修复 period=0 初始化、缺失 PWM 清理与后台测速；独立驱动 1.2.1 增加恢复轮询、结果未知、写成功但刷新失败的区分。Agent 翻译正式包异常，Studio 即时合并权威状态、防重复创建与离页污染。

fanctl 47 项、独立驱动与 Agent 199 项以及 Studio DOM 回归通过；包含正式包加载的跨层生命周期测试。详细记录见 zelux-fan/docs/fan-code-closeout-plan-20261120.md，公共语义见 zelux-protocol/docs/fan-runtime-semantics.md。

尚未完成：真机部署和统一硬件验收。接线、物理 PWM 效果和 RPM 精度不能由这些模拟测试证明；按 zelux-fan/docs/fan-closeout-hardware-test-20261120.md 验证。本次未提交、推送或操作设备。
