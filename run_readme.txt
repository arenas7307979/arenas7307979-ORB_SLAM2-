
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
   1.roscore
   2.export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/orbslam_ws/ORB_SLAM2/Examples/ROS
   3.rosrun ORB_SLAM2 RGBD Vocabulary/ORBvoc.txt Examples/RGB-D/TUM1.yaml
   4.rosbag play --pause tum_rgbd/rgbd_dataset_freiburg2_rpy.bag 
   

3. RUN rgbd(realsense) ::
   rosrun ORB_SLAM2 RGBD PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
   Example:
   1.roscore
   2.export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/orbslam_ws/ORB_SLAM2/Examples/ROS
   3.rosrun ORB_SLAM2 RGBD Vocabulary/ORBvoc.txt Examples/RGB-D/realsense.yaml
   4.rosbag play --pause d435i/indoor4.bag
   
 
