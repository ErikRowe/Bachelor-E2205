
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Joy

from pymavlink import mavutil
from pymavlink import quaternion


class BlueROV2CommNode(Node):

    def __init__(self):
        super().__init__('bluerov2_comm_node')
        self.odomPub_ = self.create_publisher(Odometry, 'CSEI/observer/odom', 10)
        self.joySub_ = self.create_subscription(Joy, 'joy', self.joystick_callback, 10)
        self.actuatorSub_ = self.create_subscription(Wrench, 'controller/output/desired_forces', self.actuation_callback, 10)
        self.actuatorSub_  # prevent unused variable warning
        timer_period = 0.001  # seconds
        self.state_update_timer = self.create_timer(timer_period, self.update_state)
        self.comm = BlueROV2Comm()

    def update_state(self):
        self.comm.read_data()
        self.publish_state()

    def publish_state(self):
        # Publish Odom msg
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "/map"
        odom_msg.child_frame_id = "/base_link"
        # Position estimate unavailable, set to 0
        odom_msg.pose.pose.position.x = float(self.comm.position[0])
        odom_msg.pose.pose.position.y = float(self.comm.position[1])
        odom_msg.pose.pose.position.z = float(self.comm.position[2])
        
        odom_msg.pose.pose.orientation.w = float(self.comm.attitude[0])
        odom_msg.pose.pose.orientation.x = float(self.comm.attitude[1])
        odom_msg.pose.pose.orientation.y = float(self.comm.attitude[2])
        odom_msg.pose.pose.orientation.z = float(self.comm.attitude[3])
        
        odom_msg.twist.twist.linear.x = float(self.comm.velocity[0])
        odom_msg.twist.twist.linear.y = float(self.comm.velocity[1])
        odom_msg.twist.twist.linear.z = float(self.comm.velocity[2])
        odom_msg.twist.twist.angular.x = float(self.comm.velocity[3])
        odom_msg.twist.twist.angular.y = float(self.comm.velocity[4])
        odom_msg.twist.twist.angular.z = float(self.comm.velocity[5])
        self.odomPub_.publish(odom_msg)
    
    def actuation_callback(self, msg):
        pwm_signals = [msg.torque.y, msg.torque.x, msg.force.z, msg.torque.z, msg.force.x, msg.force.y]
        result = []
        # Loop through and cap pwm signals at 1700/1300 (40 * 5 = 200. 1500 +- 200 = 1700 / 1300)
        for signal in pwm_signals:
            num = signal
            limit = 20
            if signal > limit: num = limit
            if signal < -limit: num = -limit
            result.append((int)(1500 + 200/limit * num))
        
        # Loop through and append desired pwm signal to channels. Unused channels are ignored
        rc_channel_values = [65535 for _ in range(18)]
        for channel_id, pwm in enumerate(result):
            rc_channel_values[channel_id] = pwm

        self.comm.set_rc_channel_pwm(rc_channel_values)

    def joystick_callback(self, msg):
        if msg.buttons[2]: # Buttons[2] = X
            self.comm.disarm()
            return
        if msg.buttons[3]: self.comm.arm() # Buttons[3] = Y
        




class BlueROV2Comm:

    def __init__(self):
        # Disable "Bare exception" warning
        # pylint: disable=W0702

        # Create the connection
        #  If using a companion computer
        #  the default connection is available
        #  at ip 192.168.2.1 and the port 14550
        # Note: The connection is done with 'udpin' and not 'udpout'.
        #  You can check in http:192.168.2.2:2770/mavproxy that the communication made for 14550
        #  uses a 'udpbcast' (client) and not 'udpin' (server).
        #  If you want to use QGroundControl in parallel with your python script,
        #  it's possible to add a new output port in http:192.168.2.2:2770/mavproxy as a new line.
        #  E.g: --out udpbcast:192.168.2.255:yourport
        
        # Using port 15000 to have QGroundControl in parallell
        self.master = mavutil.mavlink_connection('udpin:0.0.0.0:15000')

        # Make sure the connection is valid
        # This is important to have correct Mavlink message versions
        self.master.wait_heartbeat()

        self.init_stored_vars() # Initialize local variables

    def init_stored_vars(self):
        self.attitude = quaternion.QuaternionBase()
        self.position = [0]*3
        self.velocity = [0]*6
        self.previous_time = None

    def arm(self):
        # Allow thruster movement
        self.master.mav.command_long_send(
        self.master.target_system,
        self.master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)

    def disarm(self):
        #Disallow thruster movement
        self.master.mav.command_long_send(
        self.master.target_system,
        self.master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)


    def read_data(self):
        # Read sensor data from Mavlink messages
        msg = self.master.recv_match() # Intercept messages
        if not msg:
            return
        if msg.get_type() == 'ATTITUDE': # Intercept attitude messages
            msg_dict = msg.to_dict()
            
            # Only euler angle representation is available. Convert to quaternion representation
            self.velocity[3] = msg_dict['rollspeed']
            self.velocity[4] = msg_dict['pitchspeed']
            self.velocity[5] = msg_dict['yawspeed']
            self.attitude = quaternion.QuaternionBase([msg_dict['roll'], msg_dict['pitch'], msg_dict['yaw']])
            self.attitude.normalize()
        
        # If GPS is install on the BlueROV2, this function may be uncommented to recieve a position estimate
        # if msg.get_type() == 'GLOBAL_POSITION_INT':
        #     msg_dict = msg.to_dict()
        #     self.velocity[0] = msg_dict['vx']
        #     self.velocity[1] = msg_dict['vy']
        #     self.velocity[2] = msg_dict['vz']
        #     self.estimate_position(msg_dict['time_boot_ms'])

    def estimate_position(self, current_time):
        # Create position estimate from linear velocities
        if self.previous_time == None:
            self.previous_time = current_time
        dt = (current_time - self.previous_time) / 10e3
        self.position = [round(self.position[i] + self.velocity[i]*dt, 2) for i in range(3)]
        self.previous_time = current_time


    def set_rc_channel_pwm(self, rc_channel_values):
            
        self.master.mav.rc_channels_override_send(
            self.master.target_system,                # target_system
            self.master.target_component,             # target_component
            *rc_channel_values)                       # RC channel list, in microseconds.




def main(args=None):
    rclpy.init(args=args)

    comm_node = BlueROV2CommNode()

    rclpy.spin(comm_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    comm_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
