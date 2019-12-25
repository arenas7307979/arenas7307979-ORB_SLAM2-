
1. build_ros.sh fail--
   export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM2/Examples/ROS

2. RUN stereo ::
   rosrun ORB_SLAM2 Stereo PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE ONLINE_RECTIFICATION
   Example: roscore
            rosrun ORB_SLAM2 Stereo Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml true
	    rosbag play --pause V1_01_easy.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw

3. RUN rgbd(TUM) ::
   rosrun ORB_SLAM2 RGBD PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
   Example:
   roscore
   rosrun ORB_SLAM2 RGBD Vocabulary/ORBvoc.txt Examples/RGB-D/TUM1.yaml
   rosbag play --pause tum_rgbd/rgbd_dataset_freiburg2_rpy.bag /camera/depth/image:=/camera/depth_registered/image_raw /camera/rgb/image_color:=/camera/rgb/image_raw
   

3. RUN rgbd(TUM) ::
   rosrun ORB_SLAM2 RGBD PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
   Example:
   roscore
   rosrun ORB_SLAM2 RGBD Vocabulary/ORBvoc.txt Examples/RGB-D/realsense.yaml
   rosbag play --pause d435i/my_robot_indoor1.bag /camera/aligned_depth_to_color/image_raw:=/camera/depth_registered/image_raw /camera/color/image_raw:=/camera/rgb/image_raw
   

