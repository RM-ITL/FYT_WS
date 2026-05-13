# FYT_WS 自瞄效果优化指导文档

本文档只聚焦一件事：

**在不强求部署方式与 `Auto_aim_ITL` 完全相同的前提下，把 `FYT_WS` 的实际自瞄效果尽量拉到同等级。**

所以本文档不重点讨论：
- systemd
- watchdog
- 单进程入口
- udev
- 启动脚本

本文档重点讨论四类直接影响实战效果的优化：

1. 队内下位机协议适配
2. 增加速度/加速度前馈输出
3. 实现二级开火门控
4. 增加 YOLO 检测分支

并且给出：
- 每一部分怎么改
- 改哪些文件
- 为什么这样改
- 如何测试
- 如何判定“这一步是否真的起作用”

---

## 一、先说结论：`FYT_WS` 现在离 `Auto_aim_ITL` 差在哪里

从当前代码看，`FYT_WS` 的主要差距不在“有没有 EKF”，而在下面四层：

### 1. 控制输出粒度不够

当前 `GimbalCmd.msg` 只有：

- `pitch`
- `yaw`
- `yaw_diff`
- `pitch_diff`
- `distance`
- `fire_advice`

文件：
- [src/rm_interfaces/msg/GimbalCmd.msg](/home/hou/FYT_WS/src/rm_interfaces/msg/GimbalCmd.msg:1)

这意味着上位机只在给“目标角”和“误差”，没有把更细的运动趋势传给下位机。

而想接近 `Auto_aim_ITL` 的手感，至少要让下位机能吃到：
- `yaw_vel`
- `pitch_vel`
- `yaw_acc`
- `pitch_acc`

哪怕第一版只是差分出来的，也比完全没有强很多。

### 2. 当前开火判据太单层

现在 `armor_solver.cpp` 里的判火核心是：

- `isOnTarget(cur_yaw, cur_pitch, target_yaw, target_pitch, distance)`

文件：
- [src/rm_auto_aim/armor_solver/src/armor_solver.cpp](/home/hou/FYT_WS/src/rm_auto_aim/armor_solver/src/armor_solver.cpp:192)

这个逻辑的问题是：
- 只做了一个“当前误差是不是在射击窗口内”的判断
- 没有把近距离和远距离分开
- 没有做更严格的二级门控

这在实战里很容易出现：
- 近距离能打但发得保守
- 远距离窗口过松，给出误开火

### 3. 检测前端还是传统灯条主链

当前 `armor_detector` 还是：
- 灰度二值化
- 轮廓提灯条
- 灯条配对
- 数字分类

文件：
- [src/rm_auto_aim/armor_detector/src/armor_detector.cpp](/home/hou/FYT_WS/src/rm_auto_aim/armor_detector/src/armor_detector.cpp:31)

这套方法在理想光照和理想曝光下很好用，但和 `Auto_aim_ITL` 的 YOLO 主前端比，面对下面情况会吃亏：
- 背景杂乱
- 高光反射
- 局部过曝
- 遮挡
- 灯条不完整
- 高速目标拖影

### 4. 下位机协议目前没有为“效果优化”服务

当前 `ProtocolInfantry::send()` 只发：
- `fire`
- `pitch`
- `yaw`
- `distance`

文件：
- [src/rm_hardware_driver/rm_serial_driver/src/protocol/infantry_protocol.cpp](/home/hou/FYT_WS/src/rm_hardware_driver/rm_serial_driver/src/protocol/infantry_protocol.cpp:24)

也就是说：
- 就算上位机未来算出了速度/加速度
- 现在协议链也发不下去

这会直接卡死上位机控制优化的上限。

---

## 二、优化总路线

如果目标是“先把效果做起来”，建议按下面顺序推进：

### 第一优先级

1. 协议适配
2. 前馈输出
3. 二级开火门控

原因：
- 这三项是在现有检测链不变的前提下，最快能明显改善手感的部分。

### 第二优先级

4. YOLO 检测分支

原因：
- 这是效果上限的决定性因素，但开发量比前三项大，联调时间也更长。

---

## 三、优化项 1：队内下位机协议适配

## 3.1 目标

你必须先回答一个问题：

**队内下位机是否愿意接收更丰富的控制量？**

如果答案是愿意，那上位机必须把控制消息链路改成能承载这些字段：

- `control`
- `fire_advice`
- `yaw`
- `pitch`
- `yaw_vel`
- `pitch_vel`
- `yaw_acc`
- `pitch_acc`

如果下位机最终仍只收：
- `yaw`
- `pitch`
- `fire`

那你后面做的很多上位机控制优化，只能停留在“上位机内部自嗨”，很难真正体现成手感差异。

---

## 3.2 修改指导

### 第一步：改消息定义

文件：
- [src/rm_interfaces/msg/GimbalCmd.msg](/home/hou/FYT_WS/src/rm_interfaces/msg/GimbalCmd.msg:1)

建议把消息改成类似：

```text
std_msgs/Header header
bool control
bool fire_advice
float64 yaw
float64 pitch
float64 yaw_vel
float64 pitch_vel
float64 yaw_acc
float64 pitch_acc
float64 distance
```

建议：
- `yaw_diff/pitch_diff` 不作为主控制字段保留
- 如果你还想调试看误差，可以单独发布 debug topic，不要挤进主控制消息

### 第二步：让串口协议真正打通这些字段

文件：
- [src/rm_hardware_driver/rm_serial_driver/src/protocol/infantry_protocol.cpp](/home/hou/FYT_WS/src/rm_hardware_driver/rm_serial_driver/src/protocol/infantry_protocol.cpp:24)
- `src/rm_hardware_driver/rm_serial_driver/include/rm_serial_driver/protocol/infantry_protocol.hpp`

当前问题：
- `send()` 只编码了 `fire/pitch/yaw/distance`

你要做的事：

1. 和电控确定最终协议结构体。
2. 扩展发送包字段。
3. 保证 `armor_solver/cmd_gimbal` 发出的所有新字段都能进入打包逻辑。
4. 在下位机端确认接收顺序、字节序、单位制。

### 第三步：统一单位

当前 `armor_solver.cpp` 中：
- `cmd_yaw`
- `cmd_pitch`

内部大多是弧度，最后下发前转成度：
- [armor_solver.cpp](/home/hou/FYT_WS/src/rm_auto_aim/armor_solver/src/armor_solver.cpp:181)

建议：
- 明确规定消息层发给下位机到底是“度”还是“弧度”
- `yaw_vel/pitch_vel`
- `yaw_acc/pitch_acc`

也要统一：
- 如果角度是度，那速度就用 `deg/s`，加速度就用 `deg/s^2`
- 如果角度是弧度，那就全链路统一用弧度

最怕的是：
- 角度用度
- 速度用弧度
- 电控又按另一种理解

---

## 3.3 测试指导

### 协议测试 1：字段链路完整性测试

方法：
- 在 `armor_solver` 里故意输出一组可识别的固定值
- 例如：
  - `yaw = 10`
  - `pitch = 5`
  - `yaw_vel = 20`
  - `pitch_vel = 10`
  - `yaw_acc = 30`
  - `pitch_acc = 15`

然后看：
- `armor_solver/cmd_gimbal`
- 串口打包前日志
- 下位机接收值

判定标准：
- 所有字段值都能对上
- 单位一致
- 没有字段错位

### 协议测试 2：实时变化测试

方法：
- 跑视频或转动云台，让 yaw/pitch 连续变化
- 检查：
  - `yaw_vel/pitch_vel`
  - `yaw_acc/pitch_acc`

是否在下位机端也同步变化

判定标准：
- 变化方向正确
- 数值量级合理
- 没有偶发爆炸值

---

## 四、优化项 2：增加速度/加速度前馈输出

这是你列出的第二项，我认为是当前 `FYT_WS` 最值得先做的控制优化。

## 4.1 目标

最低实现允许你用连续帧差分：

```text
yaw_vel   = Δyaw / Δt
pitch_vel = Δpitch / Δt
yaw_acc   = Δyaw_vel / Δt
pitch_acc = Δpitch_vel / Δt
```

但必须处理以下 5 件事：

1. 时间戳异常
2. 第一帧无历史值
3. 目标丢失后的速度/加速度清零或安全保持
4. 突然跳变时不能输出离谱速度/加速度
5. 输出字段必须进入下位机发送链路

---

## 4.2 修改指导

### 第一步：在 `Solver` 中增加控制状态缓存

文件：
- [src/rm_auto_aim/armor_solver/include/armor_solver/armor_solver.hpp](/home/hou/FYT_WS/src/rm_auto_aim/armor_solver/include/armor_solver/armor_solver.hpp:1)

建议新增成员：

```cpp
struct ControlHistory {
  bool valid = false;
  rclcpp::Time stamp;
  double yaw = 0.0;
  double pitch = 0.0;
  double yaw_vel = 0.0;
  double pitch_vel = 0.0;
};
```

再在 `Solver` 里保存：

```cpp
ControlHistory last_control_;
```

### 第二步：在 `solve()` 尾部生成前馈量

文件：
- [src/rm_auto_aim/armor_solver/src/armor_solver.cpp](/home/hou/FYT_WS/src/rm_auto_aim/armor_solver/src/armor_solver.cpp:59)

建议在最终 `cmd_yaw/cmd_pitch` 求出来以后，加一段：

#### 1. 处理第一帧

如果 `last_control_.valid == false`
- `yaw_vel = 0`
- `pitch_vel = 0`
- `yaw_acc = 0`
- `pitch_acc = 0`

#### 2. 处理时间戳异常

如果：
- `dt <= 0`
- `dt < 1e-4`
- `dt > 0.2` 或你实际频率允许的阈值

那么：
- 直接认为这帧不适合做差分
- 速度加速度全部置零

#### 3. 计算速度

```cpp
yaw_vel = (cmd_yaw - last_yaw) / dt;
pitch_vel = (cmd_pitch - last_pitch) / dt;
```

注意：
- `yaw` 是角度量，要先统一弧度还是度
- 如果是角度环绕，必须做角度归一化

建议不要直接相减，要用：
- `angles::shortest_angular_distance()`

#### 4. 计算加速度

```cpp
yaw_acc = (yaw_vel - last_yaw_vel) / dt;
pitch_acc = (pitch_vel - last_pitch_vel) / dt;
```

#### 5. 做限幅

这一点非常重要。

建议加参数：
- `solver.max_yaw_vel`
- `solver.max_pitch_vel`
- `solver.max_yaw_acc`
- `solver.max_pitch_acc`

对计算结果做 clamp：

```cpp
yaw_vel = std::clamp(yaw_vel, -max_yaw_vel, max_yaw_vel);
pitch_vel = std::clamp(pitch_vel, -max_pitch_vel, max_pitch_vel);
yaw_acc = std::clamp(yaw_acc, -max_yaw_acc, max_yaw_acc);
pitch_acc = std::clamp(pitch_acc, -max_pitch_acc, max_pitch_acc);
```

#### 6. 目标丢失时清零

如果当前：
- `armor_target_.tracking == false`
- 或者本帧 solver 无法给出有效目标

则：
- `yaw_vel = 0`
- `pitch_vel = 0`
- `yaw_acc = 0`
- `pitch_acc = 0`

不要把上一帧的高速度原封不动带到丢失后。

#### 7. 突然跳变保护

典型触发：
- 选板切换
- 跟踪器从一块板跳到另一块板
- 目标重新捕获

建议加一个角度突变门限：
- `solver.max_delta_yaw_for_feedforward`
- `solver.max_delta_pitch_for_feedforward`

如果当前帧相对上一帧跳变超过门限：
- 当前帧速度/加速度直接清零
- 或只保留有限安全值

原因：
- 这类跳变不是目标连续运动，直接差分会算出离谱前馈

### 第三步：把新字段写进 `GimbalCmd`

文件：
- [src/rm_interfaces/msg/GimbalCmd.msg](/home/hou/FYT_WS/src/rm_interfaces/msg/GimbalCmd.msg:1)

增加：
- `yaw_vel`
- `pitch_vel`
- `yaw_acc`
- `pitch_acc`

### 第四步：保证进入发送链路

文件：
- [src/rm_hardware_driver/rm_serial_driver/src/protocol/infantry_protocol.cpp](/home/hou/FYT_WS/src/rm_hardware_driver/rm_serial_driver/src/protocol/infantry_protocol.cpp:24)

这是你明确要求的第 5 点：

**输出字段必须进入下位机发送链路**

所以只改 solver 不够，协议打包必须同步改。

---

## 4.3 测试指导

### 测试 1：静止目标稳定性测试

场景：
- 相机对着几乎不动的目标

观察：
- `yaw_vel`
- `pitch_vel`
- `yaw_acc`
- `pitch_acc`

判定：
- 应该接近 0
- 不应该大幅抖动
- 如果抖得很厉害，说明差分噪声太大，需要限幅或低通

### 测试 2：匀速摆动目标测试

场景：
- 人工慢速转云台
- 或视频中目标做平稳横向移动

判定：
- `yaw_vel/pitch_vel` 变化方向正确
- 连续性好
- 没有忽大忽小

### 测试 3：跳变保护测试

场景：
- 故意让 tracker 从一块板切到另一块板

判定：
- 不应出现极端离谱速度/加速度尖峰
- 切板瞬间前馈应清零或受限

### 测试 4：丢失目标测试

场景：
- 遮挡目标或让目标离开画面

判定：
- 丢失后前馈清零
- 下位机不会继续收到一串历史速度残留

---

## 五、优化项 3：实现二级开火门控

这是你列出的第三项，也是非常关键的一项。

## 5.1 目标

必须至少包含：

1. 当前云台 yaw/pitch
2. 目标 yaw/pitch
3. 目标距离
4. 近距离容差
5. 远距离容差
6. 距离超过阈值后使用更严格容差

当前 `isOnTarget()` 只是一个“单层射击窗口判断”，不够。

---

## 5.2 推荐设计

建议把开火拆成两层：

### 一级门控

几何上是否基本瞄上：
- 类似当前 `isOnTarget()`

### 二级门控

在一级门控通过后，再做更严格的距离相关判据：

建议新增函数：

```cpp
bool distance_based_shooter_check(
    double current_yaw,
    double current_pitch,
    double target_yaw,
    double target_pitch,
    double distance) const;
```

---

## 5.3 修改指导

### 第一步：在头文件里新增二级门控接口

文件：
- [src/rm_auto_aim/armor_solver/include/armor_solver/armor_solver.hpp](/home/hou/FYT_WS/src/rm_auto_aim/armor_solver/include/armor_solver/armor_solver.hpp:1)

新增：

```cpp
bool distance_based_shooter_check(
  double current_yaw,
  double current_pitch,
  double target_yaw,
  double target_pitch,
  double distance) const noexcept;
```

同时新增参数成员：

```cpp
double near_distance_tolerance_yaw_;
double near_distance_tolerance_pitch_;
double far_distance_tolerance_yaw_;
double far_distance_tolerance_pitch_;
double strict_distance_threshold_;
```

### 第二步：构造函数里读参数

文件：
- [src/rm_auto_aim/armor_solver/src/armor_solver.cpp](/home/hou/FYT_WS/src/rm_auto_aim/armor_solver/src/armor_solver.cpp:28)

新增参数读取：

```cpp
solver.near_tolerance_yaw
solver.near_tolerance_pitch
solver.far_tolerance_yaw
solver.far_tolerance_pitch
solver.strict_distance_threshold
```

### 第三步：实现二级门控

逻辑建议：

#### 1. 先算当前误差

```cpp
yaw_err = abs(cur_yaw - target_yaw)
pitch_err = abs(cur_pitch - target_pitch)
```

注意 yaw 用角度归一化。

#### 2. 根据距离选容差

如果：
- `distance <= strict_distance_threshold`

使用近距离容差：
- `near_tolerance_yaw`
- `near_tolerance_pitch`

否则：
- 使用更严格的远距离容差

#### 3. 最终判定

只有当：
- `yaw_err < tolerance_yaw`
- `pitch_err < tolerance_pitch`

同时满足时，二级门控才通过。

### 第四步：在 `solve()` 里串起来

当前代码：

```cpp
gimbal_cmd.fire_advice = isOnTarget(...)
```

建议改成：

```cpp
bool level1 = isOnTarget(...);
bool level2 = distance_based_shooter_check(...);
gimbal_cmd.fire_advice = level1 && level2;
```

### 第五步：不要在 `TRACKING_CENTER` 模式里直接无脑放开

当前逻辑里：

```cpp
gimbal_cmd.fire_advice = true;
```

文件：
- [armor_solver.cpp](/home/hou/FYT_WS/src/rm_auto_aim/armor_solver/src/armor_solver.cpp:167)

这一句风险很大。

建议改成：
- 即使在 `TRACKING_CENTER` 模式，也必须经过二级门控
- 最多只是在一级门控上放松，不能直接强行 `true`

---

## 5.4 测试指导

### 测试 1：近距离开火窗口测试

场景：
- 1m~3m 目标

操作：
- 轻微晃动云台

判定：
- 近距离容差可以略宽
- 不应该因为极轻微抖动完全不发火

### 测试 2：远距离开火抑制测试

场景：
- 6m 以上目标

判定：
- 远距离时 fire_advice 明显更保守
- 不应该还像近距离一样容易给开火

### 测试 3：误跟踪测试

场景：
- 目标边缘掠过准星
- 或切板瞬间

判定：
- 二级门控应阻止误发
- 不应该仅因为一级窗口短暂覆盖就给火

### 测试 4：日志测试

建议打印：
- `distance`
- `yaw_err`
- `pitch_err`
- `used tolerance`
- `level1`
- `level2`

这样你调参时知道到底是哪一层挡住了开火。

---

## 六、优化项 4：增加 YOLO 检测分支

这是决定最终效果上限的关键项。

## 6.1 目标

在保留当前传统检测器的前提下，增加一个新的 YOLO 检测分支：

- 传统分支：保底、做 fallback
- YOLO 分支：主前端

这样可以做到：
- 理想场景下 YOLO 直接出装甲板
- 复杂场景下提升鲁棒性
- 必要时保留传统分支做局部 refinement 或回退

---

## 6.2 为什么必须加

当前 `FYT_WS` 的传统前端对下面问题天然敏感：
- 阈值依赖强
- 曝光变化大时不稳
- 强反光目标容易误判
- 部分遮挡时配对容易断

而 `Auto_aim_ITL` 一类项目之所以实战效果更强，一个核心原因就是：

**装甲板前端已经不再完全依赖灯条几何配对。**

---

## 6.3 修改指导

### 第一步：先做抽象层，不要直接把 YOLO 硬塞进旧 `Detector`

文件：
- `src/rm_auto_aim/armor_detector/include/armor_detector/detector_base.hpp`
- `src/rm_auto_aim/armor_detector/include/armor_detector/traditional_detector.hpp`
- `src/rm_auto_aim/armor_detector/src/traditional_detector.cpp`

建议：
- 把当前 `Detector` 收缩成 `TraditionalDetector`
- 统一接口

当前原始文件：
- [src/rm_auto_aim/armor_detector/src/armor_detector.cpp](/home/hou/FYT_WS/src/rm_auto_aim/armor_detector/src/armor_detector.cpp:31)

### 第二步：新增 YOLO 检测器

建议新增文件：

- `src/rm_auto_aim/armor_detector/include/armor_detector/yolo_detector.hpp`
- `src/rm_auto_aim/armor_detector/src/yolo_detector.cpp`
- `src/rm_auto_aim/armor_detector/include/armor_detector/openvino_infer.hpp`
- `src/rm_auto_aim/armor_detector/src/openvino_infer.cpp`

如果你想尽量贴近 `Auto_aim_ITL`：
- 优先走 OpenVINO 部署
- 模型建议输出：
  - armor class
  - color
  - 4 keypoints

### 第三步：节点层支持 backend 选择

文件：
- [src/rm_auto_aim/armor_detector/src/armor_detector_node.cpp](/home/hou/FYT_WS/src/rm_auto_aim/armor_detector/src/armor_detector_node.cpp:217)

增加参数：

```yaml
backend: traditional
```

后续支持：
- `traditional`
- `yolo_v5`
- `yolo_v11`

### 第四步：保留当前传统分支做 fallback

建议不要一刀切删掉传统分支。

更好的结构是：

- 主链：YOLO
- fallback：TraditionalDetector

触发条件例如：
- YOLO 连续若干帧无结果
- 或 debug 时手动切换

### 第五步：如果 YOLO 没输出角点，只输出框

那你必须补一个角点恢复策略，否则 PnP 精度会差很多。

可选方案：
- 模型直接输出 4 点
- 框内局部传统几何 refinement
- 轻量角点回归头

建议优先：
- 模型直接输出 4 点

---

## 6.4 测试指导

### 测试 1：离线视频对比测试

数据源：
- 用 `video/armor_video.mp4`
- 或你们自己的录屏视频

对比项：
- 传统分支识别率
- YOLO 分支识别率
- 漏检率
- 错检率
- 输出稳定性

### 测试 2：复杂光照测试

场景：
- 高反光
- 暗场
- 背景灯干扰

判定：
- YOLO 分支识别更稳定
- 传统分支易炸的场景，YOLO 不明显掉

### 测试 3：高速运动测试

场景：
- 快速转动目标

判定：
- YOLO 分支的装甲板连续性更好
- 目标跟踪初始化更快

### 测试 4：PnP 稳定性测试

观察：
- 同一目标静止时，三维位置抖动幅度
- 同一目标移动时，轨迹是否平滑

判定：
- YOLO + 角点输出的 PnP 不应比传统更抖
- 如果更抖，说明角点定义或顺序有问题

---

## 七、推荐实施顺序

如果你现在就开改，建议按下面顺序：

### 第一阶段：先把控制链补齐

1. 改 `GimbalCmd.msg`
2. 改 `ProtocolInfantry::send()`
3. 在 `armor_solver` 里补前馈差分输出
4. 在 `armor_solver` 里补二级开火门控

原因：
- 这几步最快出效果
- 对现有检测链入侵最小

### 第二阶段：再上 YOLO 分支

5. 抽象 `Detector` 接口
6. 加 YOLO 检测器
7. 保留传统 fallback

### 第三阶段：联调参数

8. 调前馈限幅
9. 调近/远距离开火容差
10. 调 YOLO 阈值和 NMS

---

## 八、每项优化完成后的验收标准

### 协议适配完成标准

- 新字段能从 `armor_solver/cmd_gimbal` 进入串口打包链
- 下位机端能正确收值

### 前馈输出完成标准

- 静止目标时速度/加速度接近 0
- 丢失目标时清零
- 切板时不会爆离谱速度

### 二级开火门控完成标准

- 远距离明显更严格
- 误开火减少
- 近距离不过度保守

### YOLO 分支完成标准

- 复杂场景识别鲁棒性明显高于传统分支
- PnP 输出稳定
- tracker 初始化更顺滑

---

## 九、最后的建议

如果你的目标真的是“效果尽量接近 `Auto_aim_ITL`”，那开发策略上要避免两个误区：

### 误区 1

只调 `ekf` 和 `side_angle`，不动协议和控制链。

这样通常只能让系统“勉强能打”，很难让手感明显上一个层次。

### 误区 2

一上来全仓大改。

更合理的方法是：

1. 先补协议和前馈
2. 再补二级门控
3. 最后切 YOLO 主前端

这样你每一步都能看到效果变化，不会改完一大堆却不知道到底是哪一步起了作用。

