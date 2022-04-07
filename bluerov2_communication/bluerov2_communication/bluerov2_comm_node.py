
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from bluerov_interfaces.msg import ActuatorInput
from sensor_msgs.msg import Joy

from pymavlink import mavutil
from pymavlink import quaternion


class BlueROV2CommNode(Node):

    def __init__(self):
        super().__init__('bluerov2_comm_node')
        self.odomPub_ = self.create_publisher(Odometry, 'state_estimate', 10)
        self.joySub_ = self.create_subscription(Joy, 'joy', self.joystick_callback, 10)
        self.actuatorSub_ = self.create_subscription(ActuatorInput, 'actuation', self.actuation_callback, 10)
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
        odom_msg.pose.pose.position.x = 0.0
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.position.z = 0.0
        
        odom_msg.pose.pose.orientation.w = float(self.comm.attitude[0])
        odom_msg.pose.pose.orientation.x = float(self.comm.attitude[1])
        odom_msg.pose.pose.orientation.y = float(self.comm.attitude[2])
        odom_msg.pose.pose.orientation.z = float(self.comm.attitude[3])
        self.odomPub_.publish(odom_msg)
    
    def actuation_callback(self, msg):
        pwm_signals = [-msg.thrust5, msg.thrust4, msg.thrust3, -msg.thrust6, msg.thrust1, -msg.thrust2]
        result = []
        for signal in pwm_signals:
            result.append((int)(1500 + 20 * signal))

        rc_channel_values = [65535 for _ in range(18)]
        for channel_id, pwm in enumerate(result):
            rc_channel_values[channel_id] = pwm

        #for x in rc_channel_values:
        #    self.get_logger().info('Publishing: "%i"' % x)
        self.comm.set_rc_channel_pwm(rc_channel_values)

    def joystick_callback(self, msg):
        if msg.buttons[1]:
            self.comm.disarm()
            return
        if msg.buttons[0]: self.comm.arm()
        




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
        
        self.master = mavutil.mavlink_connection('udpin:0.0.0.0:15000')

        # Make sure the connection is valid
        self.master.wait_heartbeat()

        self.init_stored_vars()

    def init_stored_vars(self):
        self.attitude = quaternion.QuaternionBase()
        #self.position = [0]*3
        self.velocity = [0]*6
        self.previous_time = None

    def arm(self):
        self.master.mav.command_long_send(
        self.master.target_system,
        self.master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)

    def disarm(self):
        self.master.mav.command_long_send(
        self.master.target_system,
        self.master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)


    def read_data(self):
        msg = self.master.recv_match()
        if not msg:
            return
        if msg.get_type() == 'ATTITUDE':
            msg_dict = msg.to_dict()
            
            self.velocity[3] = msg_dict['rollspeed']
            self.velocity[4] = msg_dict['pitchspeed']
            self.velocity[5] = msg_dict['yawspeed']
            self.attitude = quaternion.QuaternionBase([msg_dict['roll'], msg_dict['pitch'], msg_dict['yaw']])
            self.attitude.normalize()
            #     if msg.get_type() == 'ATTITUDE':
            # msg_dict = msg.to_dict()
            # ACTUATOR_OUTPUT_STATUS
        # if msg.get_type() == 'GLOBAL_POSITION_INT':
        #     msg_dict = msg.to_dict()
        #     self.velocity[:3] = [msg_dict['vx'], msg_dict['vy'], msg_dict['vz']]
        #     self.estimate_position(msg_dict['time_boot_ms'])

    def estimate_position(self, current_time):
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

        # for channel_id, pwm in enumerate(rc_channel_values):
        #     self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
        #                                       mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, channel_id + 1, pwm, 0, 0, 0, 0, 0)




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
