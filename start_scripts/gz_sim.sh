
./rust_pilot gazebo_sim /home/ncer/RustPilot/sim/quadcopter.toml

./rust_pilot gazebo_actuator

./rust_pilot mixer /home/ncer/RustPilot/mixers/gz_mixer.json

./rust_pilot att_control

./rust_pilot -- manual_ctrl

./rust_pilot -- mavlink_gs --addr localhost:14550 --joystick