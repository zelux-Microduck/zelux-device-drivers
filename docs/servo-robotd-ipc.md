# 当前阶段：HD-1910滑块与中位校准（1.5.0 / API v39）

独立工厂仍为生产入口，robotd独占物理总线。servo_goal复用维护会话sync_goals，center_backup/center_servos/restore_center_backup使用新增set_offset受限事务；校准前须固定支撑并显式卸力，失败保留备份，完成后不自动加力。偏移采用符号-幅值，位置42/时间44/速度46，方向来自主控逐关节配置。ServoCenterDriver为可选扩展，不要求第三方基础驱动伪装支持校准。

配套要求：API39 robotd/robotctl、Agent1.7.0、Studio1.4.0。完整安全机制仍以zelux-robotd/docs/design/servo-studio-session.md为准。当前阶段未部署，须做单颗HD-1910硬件验收。以下为历史记录。

## 历史：支撑确认与首次调试（1.4.0 / API v38）

独立包与 Agent 内置驱动均支持 take(supported=False)，将明确支撑确认传递到 robotd，不直接打开 UART。Studio 确认后发送 true；没有确认不能绕过未归位/姿态门禁。运动、关机、总线反馈异常继续拒绝。必须配套升级 API v38 robotd/robotctl、Agent 和 Studio。

完整机制与验收以 zelux-robotd/docs/design/servo-studio-session.md 及 docs/test-plan/servo-studio-session-acceptance.md 为准。下文是上一阶段 1.3.1 / API v37 的历史记录，不是本次部署要求。

# 舵机开始调试仍报串口占用：独立驱动加载路径修复

日期：2026-11-22；驱动修复版本：1.3.1；robotd 契约：API v37。

## 根因与纠正

设备活动目录已经包含 Agent 内置 radxa_servo.py 的新 IPC 实现，不能据此认定仍在运行旧 Agent。实际报错堆栈来自 /opt/zelux/releases/1.0.0/software/zelux_drivers/radxa/servo.py 的 _open_and_ping。

RadxaHal 优先通过独立驱动包的目录工厂构造所选舵机驱动，内置实现仅为包不可用时的回退。此前只修改了内置实现；独立包仍通过 hold 文件协调旧 motiond 并直接打开 UART。robotd 正常独占 UART，因而旧独立驱动得到 EBUSY。这是漏改正式加载路径，不是要求关闭 robotd，也不是用户未更新内置 Agent 文件。

## 本次修改

- 独立驱动的生产模式改用持久 Unix socket 的 robot.servoSession，覆盖接管、快照、读写、扭矩、同步目标、标定和释放；删除旧 hold 文件协调路径，禁止退回直接打开物理串口。模拟仍使用 FakeLink。
- 对齐 API v37 的 EEPROM/危险写入确认标志、协议检查、初始快照失败释放和断连清理。
- ServoDriver 契约补充 confirmed 参数；包版本在 VERSION、pyproject 与模块元信息中统一升至 1.3.1。device-image 的 drivers 锁同步为 1.3.1，其它现有修改保留。
- 独立包测试经公开 create 工厂实例化，覆盖 socket 的 begin/snapshot/end 和 socket 不可用时不得开 UART。
- Agent 回归通过 RadxaHal 的默认真实工厂重跑相同舵机契约；不再只验证内置类。未安装独立包时这组集成回归明确跳过；独立包自己的 CI 必跑其工厂回归。

## 验证与边界

- Windows 驱动完整 pytest：29 passed；已有 asyncio_mode 配置警告，不影响本次 unittest 异步测试执行。
- Windows 与 Linux Agent 舵机专项：16 passed，包含 8 项独立包正式加载路径测试。
- Linux 独立包舵机专项：9 passed。日志位于 [servo-ipc-20261122](test-results/servo-ipc-20261122/)。
- Linux Agent 全量：173 项，171 项通过，1 项失败、1 项错误，均为风扇跨层测试在实例创建前被 PWM 已验证引脚校验拒绝；失败日志保留为 agent-tests.log。Windows 全量同两项因缺少 fcntl 报错。不把全量报告为通过，也不在本修复中放宽风扇校验。
- 本次仅 Python 驱动/契约与版本锁修改，无须重编译已有 API v37 ARM 二进制。未部署到机器人，未发送真实舵机指令，未提交或推送。

## 部署与复核

重新组装包含当前 Agent 和独立驱动 1.3.1 的设备发布包，再按既有发布流程部署。不能只复制 Agent 内置 zeluxd/hal/radxa_servo.py；真正需要更新的是活动发布目录内的 software/zelux_drivers/radxa/servo.py，并重启 Agent 使 Python 加载新实现。保留已部署的 API v37 robotd，无须为了释放串口停止它。

设备更新后先只读检查：

    sudo grep -n 'robot.servoSession' /opt/zelux/current/software/zelux_drivers/radxa/servo.py
    sudo grep -n '__version__' /opt/zelux/current/software/zelux_drivers/__init__.py
    sudo fuser -v /dev/ttyS2

确认独立驱动文件包含 IPC 调用、包版本为 1.3.1，且 UART 的唯一所有者为 robotd。机器人固定支撑后再点“开始调舵机”，核对不再调用独立驱动的直连 _open_and_ping。如果新版返回未归位/非静止安全拒绝或 robotd 版本不兼容，保留新错误继续诊断，不可恢复直连串口路径或关闭安全门禁。

设备同时报告 velstand.onnx 策略文件不存在，这是另一项部署缺件；它与本次 Agent 抢占 UART 的根因不同，不能因为舵机 IPC 修复便视为策略功能已恢复。
