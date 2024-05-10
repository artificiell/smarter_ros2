import os
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.serialization import serialize_message
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
from  tf2_msgs.msg import TFMessage
from ament_index_python.packages import get_package_share_directory
from math import radians, sin, cos
import rosbag2_py

# Logfile parser class 
class LogfileParser(Node):
    def __init__(self):
        super().__init__('logfile_parser')

        # Bag file writer
        self.writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py._storage.StorageOptions(
            uri='log_bag',
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        # Write topics meta data
        self.writer.create_topic(
            rosbag2_py._storage.TopicMetadata(
                name = '/tf',
                type = 'tf2_msgs/msg/TFMessage',
                serialization_format ='cdr'
            )
        )
        self.writer.create_topic(
            rosbag2_py._storage.TopicMetadata(
                name = '/odom',
                type = 'nav_msgs/msg/Odometry',
                serialization_format = 'cdr'
            )
        )
        self.writer.create_topic(
            rosbag2_py._storage.TopicMetadata(
                name = '/scan',
                type = 'sensor_msgs/msg/LaserScan',
                serialization_format = 'cdr'
            )
        )
        
        # Path to data log file
        self.data_file = os.path.join(
            get_package_share_directory('smarter_bag'),
            'data',
            'data.log'
        )
        self.get_logger().info(f'Parsing data log file: {self.data_file}')

    # Read and convert the logfile 
    def convert(self):
        with open(self.data_file, 'r') as f:
            for line in f:
                tokens = line.strip().split(' ')
                if tokens:
                    msg_type = tokens.pop(0)
                    if msg_type == 'sick':    
                        self.parse_sick_message(tokens)
                    elif msg_type == 'state':
                        self.parse_state_message(tokens)

    # Parse laser scan message
    def parse_sick_message(self, tokens):
        ts = float(tokens.pop(0)) if tokens else 0.0
        scan_num = int(tokens.pop(0)) if tokens else 0
        sensor_id = int(tokens.pop(0)) if tokens else 0
        range_max = float(tokens.pop(0)) if tokens else 0.0
        num_dist = int(tokens.pop(0)) if tokens else 0
        start_ang = float(tokens.pop(0)) if tokens else 0.0
        ang_inc = float(tokens.pop(0)) if tokens else 0.0
        r = [float(tok) for tok in tokens if tok]

        scan = LaserScan()
        scan.header.stamp = Time(seconds=ts).to_msg()
        scan.header.frame_id = "scan"
        scan.angle_min = radians(start_ang)
        scan.angle_max = radians(start_ang + num_dist * ang_inc)
        scan.angle_increment = radians(ang_inc)
        scan.range_min = 0.1
        scan.range_max = range_max
        scan.ranges = r[:num_dist]

        self.writer.write(
            '/scan',
            serialize_message(scan),
            Time(seconds=ts).nanoseconds)

    # Parse state and transformation messages
    def parse_state_message(self, tokens):
        ts = float(tokens.pop(0)) if tokens else 0.0
        x = float(tokens.pop(0)) if tokens else 0.0
        y = float(tokens.pop(0)) if tokens else 0.0
        a = float(tokens.pop(0)) if tokens else 0.0
        pos_known = int(tokens.pop(0)) if tokens else 0
        certainty = int(tokens.pop(0)) if tokens else 0

        quat = Quaternion()
        quat.z = sin(a / 2)
        quat.w = cos(a / 2)
        
        state_trans = TransformStamped()
        state_trans.header.stamp = Time(seconds=ts).to_msg()
        state_trans.header.frame_id = "base_link"
        state_trans.child_frame_id = "odom"
        state_trans.transform.translation.x = x
        state_trans.transform.translation.y = y
        state_trans.transform.rotation = quat
        tfmsg = TFMessage()
        tfmsg.transforms.append(state_trans)
        self.writer.write(
            '/tf',
            serialize_message(tfmsg),
            Time(seconds=ts).nanoseconds)
        
        
        state = Odometry()
        state.header.stamp = Time(seconds=ts).to_msg()
        state.header.frame_id = "odom"
        state.pose.pose.position.x = x
        state.pose.pose.position.y = y
        state.pose.pose.orientation = quat
        #state.child_frame_id = "scan"

        self.writer.write(
            '/odom',
            serialize_message(state),
            Time(seconds=ts).nanoseconds)
        
# Main function (entry point)
def main(args = None):
    rclpy.init(args = args)
    node = LogfileParser()
    node.convert()

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
