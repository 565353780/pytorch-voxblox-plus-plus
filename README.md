# Voxblox++

## Prepair the environment
```bash
sudo apt install python3-dev python3-pip python3-wstool protobuf-compiler dh-autoreconf
sudo pip3 install -U numpy protobuf scipy scikit-image ipython keras wrapt simplejson netaddr osrf-pycommon scipy pillow catkin_pkg rospkg opencv-python empy
sudo pip3 install -U tensorflow-gpu
echo "\"# voxblox++
export ROS_VERSION=noetic # (Ubuntu 16.04: kinetic, Ubuntu 18.04: melodic, Ubuntu 20.04 noetic)
export CATKIN_WS=~/catkin_ws\" >> ~/.bashrc"
```

## Init ROS workspace
```bash
mkdir -p $CATKIN_WS/src && cd $CATKIN_WS
catkin init
catkin config --extend /opt/ros/$ROS_VERSION --merge-devel
catkin config --cmake-args -DCMAKE_CXX_STANDARD=14 -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes
wstool init src
```

## Clone git package
```bash
cd src
git clone ssh://git@chLi:30001/ustc/voxblox-plusplus.git
wstool merge -t . voxblox-plusplus/voxblox-plusplus_https.rosinstall
wstool update
```

## Change the source code to support OpenCV4
```bash
bash voxblox-plusplus/update_code.sh
```

## Build the project
```bash
catkin build mask_rcnn_ros depth_segmentation gsm_node
```

## Enjoy it~
