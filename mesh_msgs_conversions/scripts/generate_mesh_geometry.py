#!/usr/bin/env python
# Lucas Walter
#
# Generate a mesh_msgs/MeshGeometryStamped message for visualizing in rviz_map_plugins/Mesh

import rospy
from geometry_msgs.msg import Point
from mesh_msgs.msg import (
    MeshGeometryStamped,
    MeshTriangleIndices,
)

if __name__ == "__main__":
    rospy.init_node("generate_mesh_geometry")

    pub = rospy.Publisher("mesh", MeshGeometryStamped, queue_size=3)
    rospy.sleep(0.5)

    msg = MeshGeometryStamped()
    msg.header.frame_id = "odom"

    for z in [-1.0, 1.0]:
        for y in [-2.0, 2.0]:
            for x in [-3.0, 3.0]:
                pt = Point(x, y, z)
                msg.mesh_geometry.vertices.append(pt)

                pt = Point(1.0, 0, 0)
                msg.mesh_geometry.vertex_normals.append(pt)

    mesh_inds = MeshTriangleIndices()
    mesh_inds.vertex_indices[0] = 0
    mesh_inds.vertex_indices[1] = 1
    mesh_inds.vertex_indices[2] = 2
    msg.mesh_geometry.faces.append(mesh_inds)

    mesh_inds = MeshTriangleIndices()
    mesh_inds.vertex_indices[0] = 4
    mesh_inds.vertex_indices[1] = 5
    mesh_inds.vertex_indices[2] = 6
    msg.mesh_geometry.faces.append(mesh_inds)

    count = 0

    # while not rospy.is_shutdown():
    if True:
        msg.header.stamp = rospy.Time.now()
        msg.uuid = f"mesh{count}"
        pub.publish(msg)
        rospy.sleep(0.5)

    rospy.spin()
