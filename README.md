# Bachelor-E2205
Repository that organizes the work of group E2205

# Controller Package

## Summary

Controller Package ROS2 node with the purpose of requiring joystick input and converting to a setpoint change for the PID. Joystick input is gathered using Joy library and then converted to body frame actions **surge, sway, heave, roll, pitch, yaw**. These actions are then converted to a setpoint change relative to current state.

The PID depends on state estimation information such as position, attitude and velocities. 

## Subscribes to
* sensor_msgs/msg/Joy Joy
* nav_msgs/msg/Odometry State_Estimate (Specific topic name unknown)

## Publishes to
* bluerov_interfaces/msg/ActuatorInput actuation

## PID

Control equation: **tau** = - Kp*v - Kd*z + g

- PID Output: **tau** = [Force_x, Force_y, Force_z, Torque_r, Torque_p, Torque_y]
- Velocities: **v** = [v_x, v_y, v_z, w_r, w_p, w_y]
- Error Vector: **z** = [x_d, y_d, z_d, sgn(w)*i_d, sgn(w)*j_d, sgn(w)*k_d]
- Position: **x** = [x, y, z]
- Attitude: **q** = [w, i, j, k]



