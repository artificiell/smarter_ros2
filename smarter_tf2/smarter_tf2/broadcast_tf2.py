import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np

# Convert Euler angles to quaternion representation.
def quaternion_from_euler(roll, pitch, yaw):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = sy * cp * sr + cy * sp * cr
    qz = sy * cp * cr - cy * sp * sr

    return np.array([qx, qy, qz, qw])

class Broadcaster(Node):

    def __init__(self):
        super().__init__('broadcast_tf')

        # Transform broadcaseter
        self.br = TransformBroadcaster(self)

        self.T1 = np.eye(4)
        self.T2 = np.eye(4)
        self.T3 = np.eye(4)

        self.T2[0:3, 3] = [1, 0, 0]
        self.T3[0:3, 3] = [1, 0, 0]

        self.T_incr = np.eye(4)
        self.T_incr[0:3, 0:3] = np.array([[np.cos(0.05), -np.sin(0.05), 0],
                                          [np.sin(0.05), np.cos(0.05), 0],
                                          [0, 0, 1]])

        # Timed callback function
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.callback)
        
    def callback(self):
        
        # Update T1, T2, and T3.
        self.T1 = np.dot(self.T1, self.T_incr)
        self.T2 = np.dot(self.T2, self.T_incr)
        self.T3 = np.dot(self.T3, self.T_incr)

        t1_stamped = TransformStamped()
        t1_stamped.header.stamp = self.get_clock().now().to_msg()
        t1_stamped.header.frame_id = 'world'
        t1_stamped.child_frame_id = 't1'
        t1_stamped.transform.translation.x = self.T1[0, 3]
        t1_stamped.transform.translation.y = self.T1[1, 3]
        t1_stamped.transform.translation.z = self.T1[2, 3]
        q = quaternion_from_euler(0, 0, np.arctan2(self.T1[1, 0], self.T1[0, 0]))
        t1_stamped.transform.rotation.x = q[0]
        t1_stamped.transform.rotation.y = q[1]
        t1_stamped.transform.rotation.z = q[2]
        t1_stamped.transform.rotation.w = q[3]
        self.br.sendTransform(t1_stamped)

        t2_stamped = TransformStamped()
        t2_stamped.header.stamp = self.get_clock().now().to_msg()
        t2_stamped.header.frame_id = 't1'
        t2_stamped.child_frame_id = 't2'
        t2_stamped.transform.translation.x = self.T2[0, 3]
        t2_stamped.transform.translation.y = self.T2[1, 3]
        t2_stamped.transform.translation.z = self.T2[2, 3]
        q = quaternion_from_euler(0, 0, np.arctan2(self.T2[1, 0], self.T2[0, 0]))
        t2_stamped.transform.rotation.x = q[0]
        t2_stamped.transform.rotation.y = q[1]
        t2_stamped.transform.rotation.z = q[2]
        t2_stamped.transform.rotation.w = q[3]
        self.br.sendTransform(t2_stamped)                                                           

        t3_stamped = TransformStamped()
        t3_stamped.header.stamp = self.get_clock().now().to_msg()
        t3_stamped.header.frame_id = 't2'
        t3_stamped.child_frame_id = 't3'
        t3_stamped.transform.translation.x = self.T3[0, 3]
        t3_stamped.transform.translation.y = self.T3[1, 3]
        t3_stamped.transform.translation.z = self.T3[2, 3]
        q = quaternion_from_euler(0, 0, np.arctan2(self.T3[1, 0], self.T3[0, 0]))
        t3_stamped.transform.rotation.x = q[0]
        t3_stamped.transform.rotation.y = q[1]
        t3_stamped.transform.rotation.z = q[2]
        t3_stamped.transform.rotation.w = q[3]
        self.br.sendTransform(t3_stamped)

# Main function
def main(args = None):
    rclpy.init(args = args)
    node = Broadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
