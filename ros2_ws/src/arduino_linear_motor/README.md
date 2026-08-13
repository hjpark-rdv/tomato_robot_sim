# Arduino Mega pin 8/9 serial control and pin 10 servo

This package relays ROS 2 Boolean topics to an Arduino Mega.  It directly sets
digital pins **8** and **9** to `HIGH` or `LOW`. Separately, the Arduino sketch
drives a servo connected to pin **10** to the angle received from ROS 2.

## Arduino

Open `arduino/mega_pin89_serial/mega_pin89_serial.ino` in Arduino IDE, select
your Arduino Mega board and its CH340 serial port, then upload. Pins 8 and 9
start `LOW` after every reset. Pin 10 does not produce servo PWM or move on boot.
It starts producing PWM only after an angle command is received and then holds
the most recently requested angle.

The sketch uses the official Arduino `Servo` library. If it is not already
available, install **Servo by Arduino** from the Arduino IDE Library Manager
before compiling. Power the servo from a supply suitable for its current
requirement and connect the supply ground to the Arduino ground; do not power a
high-current servo directly from the board's 5 V pin.

## Build and run

```bash
cd /root/farmily_tomato/ros2_ws
sudo apt install python3-serial
source /opt/ros/humble/setup.bash
colcon build --packages-select arduino_linear_motor
source install/setup.bash
ros2 launch arduino_linear_motor pin89_serial.launch.py port:=/dev/ttyUSB0
```

Use the actual port if it differs from `/dev/ttyUSB0`.

## GUI-facing ROS interface

Publish `std_msgs/msg/Bool` from the GUI:

```bash
ros2 topic pub --once /linear_motor/pin8 std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /linear_motor/pin8 std_msgs/msg/Bool '{data: false}'
ros2 topic pub --once /linear_motor/pin9 std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /linear_motor/servo10_angle_deg std_msgs/msg/Float64 '{data: 90.0}'
```

The bridge publishes connection errors and successful connection changes on
`/linear_motor/serial_status` (`std_msgs/msg/String`).
The status publisher uses transient-local durability so a GUI started after
the bridge still receives the most recent TTY connection state.

Serial protocol: `PIN 8 0`, `PIN 8 1`, `PIN 9 0`, `PIN 9 1`, or
`ANGLE 10 <10..173>`, each terminated by a newline. The angle command switches
pin 10 to the requested position and holds it.
