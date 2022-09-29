cd ..
git clone git@github.com:565353780/multi-fetch-ros.git
git clone git@github.com:565353780/robot-manage-ros.git
git clone git@github.com:565353780/grnet-detect-ros.git
git clone git@github.com:565353780/tensorboard-logger-ros.git

sudo apt install python3-dev python3-pip python3-wstool protobuf-compiler \
  dh-autoreconf ccache libpcl1-dev libpcl-dev libcgal-dev python3-catkin-tools \
  libprotobuf-dev ros-noetic-tf2-sensor-msgs ros-noetic-voxel-grid \
  ros-noetic-nav-core ros-noetic-clear-costmap-recovery ros-noetic-move-base-msgs \
  ros-noetic-rotate-recovery ros-noetic-octomap

pip3 install numpy protobuf scipy scikit-image ipython keras wrapt simplejson \
  netaddr osrf-pycommon scipy pillow catkin_pkg rospkg opencv-python empy easydict \
  tensorflow-gpu argparse easydict h5py matplotlib numpy opencv-python pyexr scipy \
  transforms3d tqdm ninja pygments open3d

cd grnet-detect-ros
./dev_setup.sh

cd ../multi-fetch-ros
./dev_setup.sh

cd ../robot-manage-ros
./dev_setup.sh

cd ../tensorboard-logger-ros
./dev_setup.sh

cd ../..
export ROS_VERSION=noetic # (Ubuntu 16.04: kinetic, Ubuntu 18.04: melodic, Ubuntu 20.04 noetic)
cp src/pytorch-voxblox-plus-plus/catkin_init.bash ./

./catkin_init.bash --none
catkin config --extend /opt/ros/$ROS_VERSION --merge-devel
catkin config --cmake-args -DCMAKE_CXX_STANDARD=17 -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes
catkin build gsm_node
cp -r src/pytorch-voxblox-plus-plus/mask_rcnn_ros/configs ~/.ros/

