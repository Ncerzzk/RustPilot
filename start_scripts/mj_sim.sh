
./target/debug/rust_pilot mujoco_sim ~/project/RustPilot/sim/mujoco_x2/scene.xml

./target/debug/rust_pilot mixer ~/project/RustPilot/mixers/mj_mixer.json

./target/debug/rust_pilot att_control

./target/debug/rust_pilot -- manual_ctrl

./target/debug/rust_pilot -- mavlink_gs --addr 127.0.0.1:14550 --joystick