# Pytorch Voxblox Plus Plus

## Prepair the environment
```bash
sudo apt install python3-dev python3-pip python3-wstool protobuf-compiler \
  dh-autoreconf ccache libpcl1-dev libpcl-dev libcgal-dev python3-catkin-tools \
  libprotobuf-dev ros-noetic-tf2-sensor-msgs ros-noetic-voxel-grid \
  ros-noetic-nav-core ros-noetic-clear-costmap-recovery ros-noetic-move-base-msgs \
  ros-noetic-rotate-recovery ros-noetic-octomap
pip3 install -U numpy protobuf scipy scikit-image ipython keras wrapt simplejson \
  netaddr osrf-pycommon scipy pillow catkin_pkg rospkg opencv-python empy open3d \
  easydict tensorflow-gpu argparse easydict h5py matplotlib numpy opencv-python \
  pyexr scipy transforms3d tqdm ninja pygments
pip3 install tensorboardX==1.2 open3d==0.10.0.0

# pip3 install -U tensorflow-gpu==2.5.0 keras-nightly==2.5.0.dev2021032900 keras==2.4.3
pip3 install torch==1.10.1+cu113 torchvision==0.11.2+cu113 torchaudio==0.10.1+cu113 \
  -f https://download.pytorch.org/whl/cu113/torch_stable.html

mkdir ~/github
cd ~/github
git clone https://github.com/facebookresearch/detectron2.git
python -m pip install -e detectron2
```

## Download models
```bash
https://dl.fbaipublicfiles.com/detectron2/COCO-InstanceSegmentation/mask_rcnn_R_101_FPN_3x/138205316/model_final_a3ec72.pkl
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
git clone https://github.com/565353780/robot-manage-ros.git
git clone https://github.com/565353780/grnet-detect-ros.git
git clone https://github.com/565353780/tensorboard_logger_ros.git
cd ..
```

## Build
```bash
cd vpp_ws
export ROS_VERSION=noetic # (Ubuntu 16.04: kinetic, Ubuntu 18.04: melodic, Ubuntu 20.04 noetic)
cp src/pytorch-voxblox-plus-plus/catkin_init.bash ./
./catkin_init.bash --none
catkin config --extend /opt/ros/$ROS_VERSION --merge-devel
catkin config --cmake-args -DCMAKE_CXX_STANDARD=17 -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes
catkin build gsm_node
cp -r src/pytorch-voxblox-plus-plus/mask_rcnn_ros/configs ~/.ros/

cd grnet-detect-ros/grnet_detect/src/GRNetDetector/extensions/chamfer_dist
python setup.py install --user

cd ../cubic_feature_sampling
python setup.py install --user

cd ../gridding
python setup.py install --user

cd ../gridding_loss
python setup.py install --user

cd ../../../../..
```

## Run
```bash
source devel/setup.zsh
roslaunch gsm_node vpp_pipeline.launch
```

### For Multi Robot
```bash
roslaunch gsm_node multi_robot_vpp_pipeline.launch
```

## Dev
```bash
git clone git@github.com:565353780/pytorch-voxblox-plus-plus.git
git clone git@github.com:565353780/multi-fetch-ros.git
git clone git@github.com:565353780/robot-manage-ros.git
git clone git@github.com:565353780/grnet-detect-ros.git
git clone git@github.com:565353780/tensorboard-logger-ros.git
```

## Enjoy it~

