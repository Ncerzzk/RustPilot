
./target/debug//rust_pilot gazebo_sim /home/ncer/RustPilot/sim/quadcopter.toml

./target/debug/rust_pilot gazebo_actuator

./target/debug/rust_pilot mixer /home/ncer/RustPilot/mixers/gz_mixer.json

#./target/debug/rust_pilot att_control

./target/debug/rust_pilot -- imu_update

#./target/debug/rust_pilot -- mavlink_gs --addr localhost:14550 --joystickW