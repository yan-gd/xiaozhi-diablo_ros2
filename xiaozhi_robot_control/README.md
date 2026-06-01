# Xiaozhi Robot Control

这个目录是独立的小智语音控制桥接功能，不修改原有 `diablo_ctrl`、`diablo_utils` 等官方代码。
本分支默认配置为 `robot2`，使用 `ROS_DOMAIN_ID=52`，不开启集群调度工具。运行配置仍统一从 `~/.config/xiaozhi_robot_control/env` 读取。
如果多台机器人在同一个局域网内同时运行，每台机器人必须使用不同的 `ROS_DOMAIN_ID`，否则同一个 `diablo/MotionCmd` 命令会被同域内所有机器人接收。

## 架构

```text
xiaozhi.me MCP 接入点
  <-> scripts/xiaozhi_mcp_pipe.py
  <-> scripts/run_robot_mcp_stdio.sh
  <-> xiaozhi_robot_control.robot_mcp_server
  <-> ROS2 /diablo/MotionCmd
  <-> diablo_ctrl_node
  <-> 下位机运动控制板
```

树莓派主动连接 `xiaozhi.me` 暴露的 MCP 接入点，所以不需要把树莓派暴露到公网。

## 暴露给小智的工具

默认 `DIABLO_ROBOT_NAME=robot2`，所以本机单独控制工具会暴露为：

- `robot2_stop`
- `robot2_move_forward(speed, duration_ms)`
- `robot2_move_backward(speed, duration_ms)`
- `robot2_turn_left(speed, duration_ms)`
- `robot2_turn_right(speed, duration_ms)`
- `robot2_raise_body(value, duration_ms)`
- `robot2_lower_body(value, duration_ms)`
- `robot2_pitch_up(value, duration_ms)`
- `robot2_pitch_down(value, duration_ms)`
- `robot2_roll_left(value, duration_ms)`
- `robot2_roll_right(value, duration_ms)`
- `robot2_reset_body_pose`
- `robot2_get_status`

只在调度机器人上设置 `DIABLO_ENABLE_CLUSTER_TOOLS=true` 时，会额外暴露集群工具。调度端在本机直接发布 robot1 命令，对 robot2/robot3 通过 UDP relay 请求它们在本机发布 `/diablo/MotionCmd`，避免跨机器 DDS 数据面不稳定导致只发现节点但命令不到达：

- `robot_cluster_stop`
- `robot_cluster_move_forward(speed, duration_ms)`
- `robot_cluster_move_backward(speed, duration_ms)`
- `robot_cluster_turn_left(speed, duration_ms)`
- `robot_cluster_turn_right(speed, duration_ms)`
- `robot_cluster_raise_body(value, duration_ms)`
- `robot_cluster_lower_body(value, duration_ms)`
- `robot_cluster_pitch_up(value, duration_ms)`
- `robot_cluster_pitch_down(value, duration_ms)`
- `robot_cluster_roll_left(value, duration_ms)`
- `robot_cluster_roll_right(value, duration_ms)`
- `robot_cluster_reset_body_pose`
- `robot_cluster_get_status`

`scripts/run_robot_mcp_stdio.sh` 默认暴露站立/趴下动作。需要关闭时设置：

```bash
export DIABLO_ENABLE_POSTURE_TOOLS=0
```

开启时会额外暴露：

- `robot2_stand_up`
- `robot2_stand_down`

同时开启集群工具和站立/趴下动作时，会额外暴露：

- `robot_cluster_stand_up`
- `robot_cluster_stand_down`

暂不默认暴露跳跃、分腿舞蹈、控制模式切换等更激烈或更容易误触发的手柄动作。

## 安全限幅

默认限幅可以通过环境变量调整：

```bash
export DIABLO_MAX_LINEAR_SPEED=0.5
export DIABLO_MAX_TURN_SPEED=0.8
export DIABLO_DEFAULT_LINEAR_SPEED=0.5
export DIABLO_DEFAULT_TURN_SPEED=0.6
export DIABLO_MAX_DURATION_MS=2000
export DIABLO_MIN_DURATION_MS=1000
export DIABLO_DEFAULT_DURATION_MS=1200
export DIABLO_DEFAULT_UP=0.0
export DIABLO_MAX_VERTICAL_SPEED=1.0
export DIABLO_DEFAULT_VERTICAL_SPEED=0.5
export DIABLO_STAND_UP_HEIGHT=1.0
export DIABLO_STAND_UP_HEIGHT_PUBLISH_MS=1200
export DIABLO_MAX_PITCH=0.5
export DIABLO_DEFAULT_PITCH=0.5
export DIABLO_MAX_ROLL=0.1
export DIABLO_DEFAULT_ROLL=0.1
```

每次移动和姿态微调工具调用都会在 `duration_ms` 后自动发布中立命令。
`robot_stand_up` 会先发送站起命令，然后按 `DIABLO_STAND_UP_HEIGHT` 继续发布高度命令，让机器人站到最大高度。

## 构建

```bash
cd ~/diablo_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select xiaozhi_robot_control
source install/setup.bash
```

## 启动机器人控制节点

`diablo_ctrl_node` 是真正和下位机运动控制板通信的节点。建议用本目录的启动脚本，它会使用当前环境里的 `ROS_DOMAIN_ID`，未设置时本分支默认为 `52`，并使用 `config/fastdds_udp_only.xml` 禁用 FastDDS 共享内存传输，避免部分 Foxy/FastDDS 环境下出现 `std::system_error: Bad address`。

多机器人同网部署时，请给每台机器人配置不同的 `ROS_DOMAIN_ID`，并确保同一台机器人上的 `diablo_ctrl_node` 和 MCP 桥接服务使用相同的值。例如：

```bash
# 机器人1
export ROS_DOMAIN_ID=51

# 机器人2
export ROS_DOMAIN_ID=52

# 机器人3
export ROS_DOMAIN_ID=53
```

仍然保持每台机器人使用不同的 `ROS_DOMAIN_ID`，并只在 `robot1` 上开启集群工具：

```bash
# 机器人1
export DIABLO_ROBOT_NAME=robot1
export ROS_DOMAIN_ID=51
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export DIABLO_ENABLE_CLUSTER_TOOLS=true
export DIABLO_CLUSTER_ROS_DOMAIN_IDS=51,52,53
export DIABLO_CLUSTER_ROBOT_NAMES=robot1,robot2,robot3
export DIABLO_CLUSTER_PRESTART=true
export DIABLO_CLUSTER_CALL_RETRY_COUNT=2
export DIABLO_CLUSTER_RELAY_KEY=replace-with-random-shared-key
export DIABLO_CLUSTER_UDP_TARGETS=robot2=10.131.89.204:8765,robot3=10.131.89.63:8765
export DIABLO_CLUSTER_REQUIRE_SUBSCRIBER=false
export DIABLO_CLUSTER_REQUIRE_ALL_READY=false
export DIABLO_CLUSTER_READY_RETRY_COUNT=3
export DIABLO_WAIT_FOR_SUBSCRIBER_MS=15000
export DIABLO_DISCOVERY_SETTLE_MS=2000

# 机器人2
export DIABLO_ROBOT_NAME=robot2
export ROS_DOMAIN_ID=52
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export DIABLO_ENABLE_CLUSTER_TOOLS=false
export DIABLO_UDP_RELAY_LISTEN=true
export DIABLO_UDP_RELAY_PORT=8765
export DIABLO_UDP_RELAY_KEY=replace-with-random-shared-key

# 机器人3
export DIABLO_ROBOT_NAME=robot3
export ROS_DOMAIN_ID=53
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export DIABLO_ENABLE_CLUSTER_TOOLS=false
export DIABLO_UDP_RELAY_LISTEN=true
export DIABLO_UDP_RELAY_PORT=8765
export DIABLO_UDP_RELAY_KEY=replace-with-random-shared-key
```

`robot_cluster_*` 工具由 `robot1` 统一接收。robot1 本机直接执行 robot1 的动作；远端机器人由 UDP relay 接收带签名的 JSON 请求，并在远端本机调用同一套单机器人工具。`DIABLO_CLUSTER_RELAY_KEY` 和 `DIABLO_UDP_RELAY_KEY` 必须一致；不要使用示例值。这样单机控制仍由 `robot1_*`、`robot2_*`、`robot3_*` 完成，群控只通过 `robot_cluster_*` 入口完成。

```bash
cd ~/diablo_ws/src/xiaozhi_robot_control
bash ./scripts/run_diablo_ctrl_node_udp.sh
```

如果需要后台运行：

```bash
systemd-run --user --unit=diablo-ctrl-node --description=diablo-ctrl-node \
  bash ~/diablo_ws/src/xiaozhi_robot_control/scripts/run_diablo_ctrl_node_udp.sh
```

## 开机自启

安装并启用单一的用户 systemd 服务：

```bash
cd ~/diablo_ws/src/xiaozhi_robot_control
bash ./scripts/install_simple_startup.sh
```

本服务（`xiaozhi-diablo-startup`）只从 `~/.config/xiaozhi_robot_control/env` 读取运行配置。不要把 `MCP_ENDPOINT`、`ROS_DOMAIN_ID`、`DIABLO_*` 同时写在 `~/.bashrc` 里，避免 systemd 服务和交互终端使用不同配置。

如果还未配置，先复制示例文件并填写 `MCP_ENDPOINT`，然后使用 `systemctl --user restart xiaozhi-diablo-startup.service` 重启服务即可生效：

```bash
mkdir -p ~/.config/xiaozhi_robot_control
cp ~/diablo_ws/src/xiaozhi_robot_control/config/xiaozhi_robot_control.env.example \
  ~/.config/xiaozhi_robot_control/env
nano ~/.config/xiaozhi_robot_control/env
```

如果需要未登录桌面也能开机启动用户服务，执行一次：

```bash
sudo loginctl enable-linger diablo
```

查看状态和日志：

```bash
systemctl --user status xiaozhi-diablo-startup.service
journalctl --user -u xiaozhi-diablo-startup.service -f
```

## 本地测试 MCP 工具

```bash
cd ~/diablo_ws/src/xiaozhi_robot_control
python3 ./scripts/local_mcp_smoke_test.py
```

这个测试会初始化 MCP、列出工具，并默认调用一次 `robot2_stop`。服务也兼容内部旧名 `robot_stop`。它只发布 ROS2 消息，不会直接操作串口。

## 连接 xiaozhi.me

在小智后台复制 MCP 接入点，填入 `~/.config/xiaozhi_robot_control/env`：

```bash
nano ~/.config/xiaozhi_robot_control/env
```

本目录已经内置了一个轻量版 `scripts/xiaozhi_mcp_pipe.py`，不需要额外准备官方 `mcp_pipe.py`。

```bash
export ROS_DOMAIN_ID=52
cd ~/diablo_ws/src/xiaozhi_robot_control
bash ./scripts/run_with_xiaozhi_endpoint.sh
```

启动后到 `xiaozhi.me` 后台刷新 MCP 状态。在线后，小智就可以调用本目录暴露的 `robot_*` 工具。

查看后台日志：

```bash
journalctl --user -u xiaozhi-diablo-startup.service -f
```

## 推荐语音说法

- “小智，让机器人停下”
- “小智，让机器人向前走一秒”
- “小智，让机器人向后退半秒”
- “小智，让机器人左转一下”
- “小智，让机器人抬高一点”
- “小智，让机器人降低一点”
- “小智，让机器人抬头一点”
- “小智，让机器人低头一点”
- “小智，让机器人向左倾斜一点”
- “小智，让机器人向右倾斜一点”
- “小智，让机器人姿态回正”
- “小智，让机器人站起来”
- “小智，让机器人趴下”
- “小智，查询机器人状态”
