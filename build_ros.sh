echo "Building ROS nodes"
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/orbslam_ws/ORB_SLAM2/Examples/ROS

rm -r Examples/ROS/ORB_SLAM2/build/

cd Examples/ROS/ORB_SLAM2
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j


