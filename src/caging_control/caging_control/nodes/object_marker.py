"""
Draw the object in RViz (Step 5).

RViz cannot read Gazebo (SDF) models, so it does not show the object by itself.
This node publishes a Marker with the same shape and color as the object SDF,
attached to the TF frame "object" (world -> object comes from the Gazebo ground
truth), so RViz draws it exactly where the object really is.

  box        -> a CUBE of 0.2 m (its center is 0.1 m above the floor)
  hemisphere -> the mesh caging_description/meshes/hemisphere.stl
  pyramid    -> the mesh caging_description/meshes/pyramid.stl

Publishes:
  /object/marker  (visualization_msgs/Marker)   once per second

Parameters:
  object   box, hemisphere or pyramid    (default: box)

Run:
  ros2 run caging_control object_marker --ros-args -p object:=hemisphere
"""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

# Colors (r, g, b) of the objects, the same as in caging_description/objects/*.sdf
COLORS = {
    'box': (0.8, 0.2, 0.2),
    'hemisphere': (0.2, 0.3, 0.8),
    'pyramid': (0.2, 0.7, 0.3),
}


class ObjectMarker(Node):

    def __init__(self):
        super().__init__('object_marker')
        self.declare_parameter('object', 'box')
        self.object = self.get_parameter('object').value
        if self.object not in COLORS:
            self.get_logger().error(f'Unknown object "{self.object}": use box, hemisphere or pyramid')
            self.object = 'box'

        self.marker_pub = self.create_publisher(Marker, '/object/marker', 10)
        # The shape never changes, but RViz may start later than this node:
        # publishing once per second makes sure it receives the marker.
        self.create_timer(1.0, self.publish_marker)
        self.get_logger().info(f'Drawing the {self.object} on /object/marker (frame "object")')

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = 'object'   # moves with the object (TF from the ground truth)
        # stamp = 0: "use the newest transform of the frame"
        marker.ns = 'object'
        marker.id = 0
        marker.action = Marker.ADD
        # frame_locked = True: RViz moves the marker with the TF frame "object" in EVERY
        # frame. With False (the default) RViz places it once, when the message arrives
        # (here once per second), so a carried object would jump every second (Step 8).
        marker.frame_locked = True

        if self.object == 'box':
            marker.type = Marker.CUBE
            marker.pose.position.z = 0.1      # the frame is on the floor, the cube center is not
            marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        else:
            marker.type = Marker.MESH_RESOURCE
            marker.mesh_resource = f'package://caging_description/meshes/{self.object}.stl'
            marker.scale.x = marker.scale.y = marker.scale.z = 1.0   # the mesh is in meters
        marker.pose.orientation.w = 1.0

        r, g, b = COLORS[self.object]
        marker.color.r, marker.color.g, marker.color.b = r, g, b
        marker.color.a = 1.0
        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = ObjectMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
