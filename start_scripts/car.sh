./rust_pilot --server &

sleep 1

./rust_pilot -- elrs  /dev/ttyS2top
# set spi mode tempolarily, as it's a FPGA module bug.
# I have fix the bug in dshot mode, while pwm mode i have never test yet
# so just set spi-mode to 0, in which the module could work
./rust_pilot -- fpga_spi_pwm -d /dev/spidev0.2 --predivider 4 -f ../fpga/spi_pwm.bin --spi-mode 0
./rust_pilot -- mixer ./car_mixer.toml
./rust_pilot -- manual_ctrl -d
