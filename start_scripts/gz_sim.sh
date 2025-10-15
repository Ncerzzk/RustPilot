
./target/debug/rust_pilot gazebo_sim ~/project/RustPilot/sim/quadcopter.toml

./target/debug/rust_pilot gazebo_actuator

./target/debug/rust_pilot mixer ~/project/RustPilot/mixers/gz_mixer.json

./target/debug/rust_pilot att_control

./target/debug/rust_pilot -- manual_ctrl

./target/debug/rust_pilot -- mavlink_gs --addr 127.0.0.1:14550 --joystick
