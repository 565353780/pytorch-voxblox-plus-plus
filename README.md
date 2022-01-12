# Voxblox++

## Prepair the environment
```bash
sudo apt install python3-dev python3-pip python3-wstool protobuf-compiler dh-autoreconf ccache libpcl1-dev libpcl-dev
sudo pip3 install -U numpy protobuf scipy scikit-image ipython keras wrapt simplejson netaddr osrf-pycommon scipy pillow catkin_pkg rospkg opencv-python empy
# sudo pip3 install -U tensorflow-gpu==2.5.0 keras-nightly==2.5.0.dev2021032900 keras==2.4.3
python -m pip install 'git+https://github.com/facebookresearch/detectron2.git'
echo "\"# voxblox++
export ROS_VERSION=noetic # (Ubuntu 16.04: kinetic, Ubuntu 18.04: melodic, Ubuntu 20.04 noetic)
export CATKIN_WS=~/catkin_ws\" >> ~/.bashrc"
```

## Init ROS workspace
```bash
mkdir -p $CATKIN_WS/src && cd $CATKIN_WS
catkin init
catkin config --extend /opt/ros/$ROS_VERSION --merge-devel
catkin config --cmake-args -DCMAKE_CXX_STANDARD=17 -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes
```

## Build the project
```bash
cd src
git clone https://github.com/565353780/voxblox-plus-plus.git
cd ..
catkin build gsm_node
cp src/voxblox-plus-plus/catkin_init.bash ./
./catkin_init.bash --none
```

## Enjoy it~

