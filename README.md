# Joey: Customizable ROS Joystick Driver for Differential-Drive Robots

Joey is a highly-customizable ROS joystick driver designed for differential-drive robots. Users are able to customize axis and buttons for linear/angular velocity, linear/angular boost(acceleration) and safety(shutdown) mode.

## Install

### Dependencies

`Joey` depends on ROS default joystick driver, you can install it via `sudo apt install ros-<distr>-joy`.

### Clone and compile

~~~
cd <ros_workspace>/src
git clone https://github.com/MuchenSun/Joey.git
cd ..
catkin build
~~~

## Usage

The `Joey_node` ROS node will subscribe to a joystick topic (with `sensor_msgs::Joy` message) from the `joy_node` node in the `joy` package (installation step above), and publish a velocity command in the format of `geometry_msgs::Twist` on the specified topic.

There are two example launch files named "sony\_dualshock3.launch" and "logitech\_f310.launch" specified for two common joystick models. They are a good start for customizing your own joystick driver.

### Parameters

 - joy\_topic: Joystick topic to subscribe to.
 - cmd\_topic: Control topic to publish to.
 - max\_lin\_vel: Maximum linear velocity allowed.
 - max\_ang\_vel: Maximum angular velocity allowed.
 - lin\_boost\_ratio: When not boosted, the maximum achievable linear velocity would be (max\_lin\_vel)x(lin\_boost\_ratio), the true maximum velocity could only be achieved with full boost.
 - ang\_boost\_ratio: When not boosted, the maximum achievable angular velocity would be (max\_ang\_vel)x(ang\_boost\_ratio), the true maximum velocity could only be achieved with full boost.
 - fractional\_boost: Whether the joystick supports (for example, whether the L1/2 and R1/2 buttons support fractional input).
 - lin\_vel\_axis: Joystick axis for linear velocity control.
 - lin\_boost\_axis: Joystick axis for linear boost control.
 - ang\_vel\_axis: Joystick axis for angular velocity control.
 - ang\_boost\_axis: Joystick axis for angular boost control. 
 - shutdown\_axis: Joystick axis for enabling "shutdown" mode, in which all other inputs will be ignored and the driver will publish zero-velocity command.
 - recover\_axis: Joystick axis for recovering from "shutdown" mode.

## License

This project is licensed under GPLv3. For any question, feel free to contact Muchen Sun (sunmch15@gmail.com).
