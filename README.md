
## RustPilot
一个运行于Linux上的飞控

## Features
- 保持简单，目标群体是 <= 250g的 微型无人机/航模
- 测试（仿真）驱动，大部分功能能够在仿真中调试
- 组件式设计，飞控各个组件：传感器-融合-控制-混控-输出 分开设计，因此未来可以很方便地替换任一部分
- Gazebo 仿真支持（同时支持以gazebo 时钟作为飞控运行时钟(lock step))

## Progress
- [x] 支持Gazebo仿真基本的四旋翼飞行器(单角度环控制)
- [ ] 增加角速度环
- [ ] 四旋翼样机测试
- [ ] 增加固定翼机型支持
- [ ] gps导航支持

## Build

```
git clone https://github.com/Ncerzzk/RustPilot.git
git submodule update --init --recursive
cd RustPilot
cargo build

or 

cargo build --features gzsim  # if you want to run gazebo sim(install gazebo harmonic first)
```

