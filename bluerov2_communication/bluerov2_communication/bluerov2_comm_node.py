
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from bluerov_interfaces.msg import ActuatorInput

from bluerov2_comm import BlueROV2Comm


class BlueROV2CommNode(Node):

    def __init__(self):
        super().__init__('bluerov2_comm_node')
        self.odomPub_ = self.create_publisher(Odometry, 'state_estimate', 10)
        timer_period = 0.1  # seconds
        self.state_update_timer = self.create_timer(timer_period, self.update_state)
        self.comm = BlueROV2Comm()

    def update_state(self):
        self.comm.read_data()
        self.publish_state()

    def publish_state(self):
        # Publish Odom msg
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now()
        odom_msg.header.frame_id = "/map"
        odom_msg.child_frame_id = "/base_link"
        odom_msg.pose.pose.position.x = 0
        odom_msg.pose.pose.position.y = 0
        odom_msg.pose.pose.position.z = 0
        
        odom_msg.pose.pose.orientation.w = self.comm.attitude[0]
        odom_msg.pose.pose.orientation.x = self.comm.attitude[1]
        odom_msg.pose.pose.orientation.y = self.comm.attitude[2]
        odom_msg.pose.pose.orientation.z = self.comm.attitude[3]
        self.odomPub.publish(odom_msg)
    
    def subscribe_actuation(self):
        return


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
