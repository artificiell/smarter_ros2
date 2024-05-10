import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_ros.buffer_interface import TransformStamped
from visualization_msgs.msg import Marker
import numpy as np
import tf2_ros
from tf2_geometry_msgs import PoseStamped

def TFToMarkerArrow(transform, ns, frame_id, color):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rclpy.time.Time().to_msg()
    marker.ns = ns
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD

    
    # Convert transform to Pose
    pose = Pose()
    pose.position.x = transform.transform.translation.x
    pose.position.y = transform.transform.translation.y
    pose.position.z = transform.transform.translation.z
    pose.orientation.x = transform.transform.rotation.x
    pose.orientation.y = transform.transform.rotation.y
    pose.orientation.z = transform.transform.rotation.z
    pose.orientation.w = transform.transform.rotation.w    
    marker.pose = pose

    # Set marker size and color
    marker.scale.x = 1.0
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0
    if color == 'red':
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
    elif color == 'green':
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
    else:
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        
    return marker                                                                                                                

class DebugMarker(Node):
    
    def __init__(self):
        super().__init__('debug_marker')
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timed callback function
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.callback)

    def query_transform(self, parent, child):
        try:
            transform = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
            return transform
        except  TransformException as ex:
            self.get_logger().error(f'Transform error: {ex}')
        return None

    def callback(self):

        # Query the /tf2 and find the pose between different frames and draw an arrow
        try:
            transform = self.query_transform('world', 't2')
            if transform:
                marker = TFToMarkerArrow(transform, 't2', 'world', 'red')
                self.marker_pub.publish(marker)            
            transform = self.query_transform('world', 't3')
            if transform:
                marker = TFToMarkerArrow(transform, 't3', 'world', 'green')
                self.marker_pub.publish(marker)
        except Exception as ex:
            self.get_logger().error(f'Marker error: {ex}')

# Main function           
def main(args = None):
    rclpy.init(args = args)
    node = DebugMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
