# Bachelor-E2205
Repository that organizes the work of group E2205

# Controller Package

## Subscribes to
* sensor_msgs/msg/Joy Joy
* nav_msgs/msg/Odometry State_Estimate (Specific topic name unknown)

## Publishes to
* bluerov_interfaces/msg/ActuatorInput actuation

# PID

Requires information regarding position **x** = [x, y, z], attitude **q** = [w, i, j, k] and velocities **v** = [vx, vy, vz, wx, wy, wz]


# Simen
## Install dependecies
sudo apt install wiringpi
sudo apt-get install libwiringpi-dev

## Maybe these
sudo apt-get install libi2c-dev
gpio load i2c
