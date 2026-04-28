# Manipulator Component

## 项目简介

通用机械臂控制库，统一 C API，支持多种硬件设备通过驱动插件接入。

## 代码结构

```
manipulator/
├── include/
│   ├── manipulator.h                    # 公共 API 接口
│   ├── kinematics_interface.h           # 运动学求解器接口
│   └── so101_utils.h                    # SO-101 工具定义
├── src/
│   ├── manipulator.c                    # 核心实现（驱动注册、设备管理）
│   ├── manipulator_core.h               # 内部头文件
│   ├── drivers/
│   │   ├── drv_dummy.c                  # Dummy 驱动（测试/占位）
│   │   └── drv_uart_so101.c             # SO-101 UART 驱动
│   └── kinematics/
│       ├── kinematics.c                 # 求解器注册框架
│       ├── kinematics_dummy.c           # Dummy 求解器
│       └── kinematics_pinocchio.cpp     # Pinocchio 后端
├── urdf/
│   └── so101.urdf                       # SO-101 简化 URDF
├── test/
│   ├── test_manipulator.c               # 单元测试
│   ├── test_kinematics.c                # 运动学测试
│   └── test_hw_so101.c                  # 硬件交互测试
├── CMakeLists.txt                       # 构建配置
├── package.xml                          # 依赖声明
├── LICENSE                              # Apache-2.0 许可证
├── NOTICE                               # 第三方归属声明
└── README.md                            # 本文档
```

## 功能特性

- 驱动插件架构：自动注册，无需修改核心代码即可扩展新硬件
- 运动学集成：FK/IK 通过求解器框架接入（支持 Pinocchio 等后端）
- 关节运动 (PTP) 和直线运动 (Line) 接口
- 校准流程：零位偏移 + 行程范围记录
- 线程安全状态管理

### 已支持硬件

| 驱动      | 文件                         | 说明                               |
|-----------|------------------------------|------------------------------------|
| `dummy`   | `src/drivers/drv_dummy.c`    | 测试驱动，模拟即时完成             |
| `so101`   | `src/drivers/drv_uart_so101.c` | SO-101 5-DOF，Feetech STS3215 舵机 |

### 运动学后端

| 后端        | 编译选项                 | 特点                        |
|-------------|--------------------------|-----------------------------|
| `dummy`     | 默认编译                 | 占位，返回 `KIN_ERR_NOSYS` |
| `pinocchio` | `MANIP_ENABLE_PINOCCHIO` | FK/IK 完整实现，CLIK 数值  |

## 快速开始

### 依赖清单

| 依赖      | 必需/可选 | 安装方式                           |
|-----------|-----------|-------------------------------------|
| pthreads  | 必需      | 系统自带                           |
| libm      | 必需      | 系统自带                           |
| pinocchio | 可选      | 源码编译（见下文）                 |
| motor     | 可选      | `components/peripherals/motor`     |
| grasp     | 可选      | `components/control/grasp`         |

### Pinocchio 源码编译

1. 安装基础依赖：

```bash
sudo apt install -y cmake build-essential libeigen3-dev libboost-all-dev liburdfdom-dev
```

2. 编译安装 coal（碰撞检测库，可选）：

```bash
git clone --recursive -b v3.0.2 https://github.com/coal-library/coal.git
cd coal
mkdir build && cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_PYTHON_INTERFACE=OFF \
    -DBUILD_TESTING=OFF
make -j$(nproc)
sudo make install
sudo ldconfig
```

3. 编译安装 pinocchio：

```bash
git clone --recursive -b v3.9.0 https://github.com/stack-of-tasks/pinocchio.git
cd pinocchio
mkdir build && cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_PYTHON_INTERFACE=OFF \
    -DBUILD_TESTING=OFF \
    -DBUILD_WITH_COLLISION_SUPPORT=OFF
make -j$(nproc)
sudo make install
sudo ldconfig
```

> 注：如果只需要 FK/IK 功能，可跳过 coal 编译，并设置 `-DBUILD_WITH_COLLISION_SUPPORT=OFF`。

### 构建编译

```bash
mkdir build && cd build
cmake ..
make
```

完整编译（SO-101 + Pinocchio）：

```bash
mkdir build && cd build
cmake .. -DMANIP_BUILD_HW_TEST=ON
make -j$(nproc)
```

> 源码编译的 Pinocchio 安装到 `/usr/local` 后，`find_package(pinocchio)` 可自动找到，无需额外设置环境变量。

CMake 选项：

| 选项                        | 默认 | 说明                |
|-----------------------------|------|---------------------|
| `MANIP_ENABLE_SO101_DRIVER` | ON   | SO-101 UART 驱动    |
| `MANIP_ENABLE_PINOCCHIO`    | ON   | Pinocchio 运动学    |
| `MANIP_BUILD_TESTS`         | ON   | 单元测试            |
| `MANIP_BUILD_HW_TEST`       | OFF  | SO-101 硬件交互测试 |

### 运行示例

```c
#include "manipulator.h"

struct manip_dev *arm = manip_alloc("so101", NULL);

manip_joint_t target = {
    .count = 5,
    .joints = {0.0f, -0.5f, 0.5f, 0.0f, 0.0f}
};
manip_move_joints(arm, &target, 1.0f);

manip_stop(arm);
manip_free(arm);
```

## 详细使用

### 公共 API (`manipulator.h`)

#### 设备生命周期

```c
struct manip_dev *manip_alloc(const char *driver_name, void *args);
void manip_free(struct manip_dev *dev);
```

#### 运动控制

```c
int manip_move_joints(struct manip_dev *dev, const manip_joint_t *target, float speed_ratio);
int manip_move_line(struct manip_dev *dev, const manip_pose_t *target, float speed_ratio);
void manip_stop(struct manip_dev *dev);
```

#### 状态查询

```c
int manip_get_state(struct manip_dev *dev, manip_joint_t *joints, manip_pose_t *pose);
void manip_tick(struct manip_dev *dev, float dt_s);
```

#### 运动学绑定

```c
int manip_set_kinematics(struct manip_dev *dev, struct kin_solver *solver);
```

> 调用后 solver 所有权转移给 dev；`manip_free()` 会自动释放。

#### 错误码

| 宏                  | 值   | 含义         |
|---------------------|------|--------------|
| `MANIP_OK`          |  0   | 成功         |
| `MANIP_ERR_ALLOC`   | -1   | 内存分配失败 |
| `MANIP_ERR_CONNECT` | -2   | 通信失败     |
| `MANIP_ERR_TIMEOUT` | -3   | 超时         |
| `MANIP_ERR_CONFIG`  | -4   | 配置错误     |
| `MANIP_ERR_PARAM`   | -5   | 参数错误     |
| `MANIP_ERR_NOSYS`   | -6   | 功能未实现   |

### 运动学求解器 (`kinematics_interface.h`)

```c
kin_solver_t *kin_create(const char *solver_name,
                         const char *urdf_path,
                         const char *base_link,
                         const char *tip_link);
void kin_destroy(kin_solver_t *solver);

int kin_forward(kin_solver_t *s,
                const kin_joints_t *joints,
                kin_pose_t *out);
int kin_inverse(kin_solver_t *s,
                const kin_pose_t *target,
                const kin_joints_t *q_init,
                const kin_ik_params_t *params,
                kin_joints_t *out);
```

### 扩展新驱动

1. 在 `src/drivers/` 中新增 `.c` 文件
2. 包含 `"../manipulator_core.h"`
3. 实现 `struct manip_ops` 操作表和工厂函数
4. 使用 `REGISTER_MANIP_DRIVER("name", factory)` 注册
5. 在 `CMakeLists.txt` 中添加源文件
6. 用户通过 `manip_alloc("name", args)` 选用

### 扩展新求解器

1. 在 `src/kinematics/` 中新增 `.c` / `.cpp` 文件
2. 实现 `struct kin_ops` 操作表和工厂函数
3. 使用 `REGISTER_KIN_SOLVER("name", factory)` 注册
4. 在 `CMakeLists.txt` 中添加源文件及依赖库

## 常见问题

**Q: 编译时找不到 Pinocchio？**
确保已按照「环境准备」章节完成 Pinocchio 源码编译安装，并执行了 `sudo ldconfig` 更新库缓存。如安装到非默认路径，需在 cmake 时指定 `-DCMAKE_PREFIX_PATH=<安装路径>`。

**Q: SO-101 连接失败？**
检查串口权限（`sudo chmod 666 /dev/ttyACM0`）和波特率（默认 1000000）。

**Q: 校准后零位不准？**
确认校准时手动将各关节移到了**精确中位**，确认 `config/so101_calibration.json` 文件正确保存。

## 版本与发布

版本以本组件文档或仓库 tag 为准。

| 版本  | 说明                                                   |
|-------|--------------------------------------------------------|
| 1.0.0 | 初始版本，支持 SO-101 驱动，Pinocchio 运动学，夹爪控制 |

## 贡献方式

欢迎参与贡献：提交 Issue 反馈问题，或通过 Pull Request 提交代码。

1. C/C++ 代码遵循 [Google C++ 风格](https://google.github.io/styleguide/cppguide.html)
2. Python 代码遵循 [PEP 8](https://peps.python.org/pep-0008/)
3. Git commit 遵循 [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/)

## License

本组件源码文件头声明为 Apache-2.0，最终以本目录 `LICENSE` 文件为准。