# rpg-ros
Custom ROS code for RPG needs, including:
1. A driver for the Jaguar motion base
2. roscam, a bridge between HAL and ROS for imagery
3. ros2node, a bridge between ROS to Node for imagery

Currently, ros2node only accepts a pair of ROS topics to schlep together into a PbMsgs::ImageMsg for publication and consumption by Node clients.

roslaunch ros2node ros2node_node_ros_topic:=/camera/image/rgb_raw|/camera/image/depth_raw _node_topic:=images

will connect to the two ROS topics (separated by a pipe) and slave the output to Node available on ros2node/images

