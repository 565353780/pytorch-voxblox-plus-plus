# Voxblox++

## Prepair the environment
```bash
sudo apt install python3-dev python3-pip python3-wstool protobuf-compiler dh-autoreconf ccache libpcl1-dev libpcl-dev
pip3 install -U numpy protobuf scipy scikit-image ipython keras wrapt simplejson netaddr osrf-pycommon scipy pillow catkin_pkg rospkg opencv-python empy
# pip3 install -U tensorflow-gpu==2.5.0 keras-nightly==2.5.0.dev2021032900 keras==2.4.3
pip3 install -U tensorflow-gpu
python -m pip install 'git+https://github.com/facebookresearch/detectron2.git'
```

## Download models
```bash
https://dl.fbaipublicfiles.com/detectron2/COCO-InstanceSegmentation/mask_rcnn_R_101_FPN_3x/138205316/model_final_a3ec72.pkl ~/.ros/
https://gateway.infinitescript.com/?fileName=GRNet-ShapeNet.pth
```

and save them to
```bash
~/.ros/
```

## Clone
```bash
mkdir -p vpp_ws/src
cd vpp_ws/src
git clone https://github.com/565353780/pytorch-voxblox-plus-plus.git
git clone https://github.com/565353780/multi-fetch-ros.git
git clone https://github.com/565353780/grnet-detect-ros.git
git clone https://github.com/565353780/tensorboard_logger_ros.git
cd ..
```

## Build
```bash
cd vpp_ws
export ROS_VERSION=noetic # (Ubuntu 16.04: kinetic, Ubuntu 18.04: melodic, Ubuntu 20.04 noetic)
catkin init
catkin config --extend /opt/ros/$ROS_VERSION --merge-devel
catkin config --cmake-args -DCMAKE_CXX_STANDARD=17 -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes
catkin build gsm_node
cp src/pytorch-voxblox-plus-plus/catkin_init.bash ./
cp -r src/pytorch-voxblox-plus-plus/mask_rcnn_ros/configs ~/.ros/
./catkin_init.bash --none
```

## Dev
```bash
git clone git@github.com:565353780/pytorch-voxblox-plus-plus.git
git clone git@github.com:565353780/multi-fetch-ros.git
git clone git@github.com:565353780/grnet-detect-ros.git
git clone git@github.com:565353780/tensorboard-logger-ros.git
```

## Enjoy it~

