#!/usr/bin/env python
import rospy
import roslaunch
import os

def start_nodes():
    # Initialize the ROS node
    rospy.init_node('my_mapping_launcher', anonymous=True)

    # Path to your mapping package
    mapping_package_path = rospy.get_param("mapping_package_path", "/home/dhruv/catkin_ws/src/mapping")

    # Load the gmapping node 
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    gmapping_launch = os.path.join(mapping_package_path, "launch", "gmapping.launch")
    gmapping_node = roslaunch.parent.ROSLaunchParent(uuid, [gmapping_launch])
    gmapping_node.start()

    # Load the rviz configuration
    rviz_config = os.path.join(mapping_package_path, "rviz", "gmapping.rviz")
    cli_args = ['rviz', 'rviz', '-d', rviz_config]
    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
    rviz_node = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    rviz_node.start()

    # Keep the script alive
    rospy.spin()

if __name__ == '__main__':
    try:
        start_nodes()
    except rospy.ROSInterruptException:
        pass
