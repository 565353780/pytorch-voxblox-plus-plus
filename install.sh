sudo apt install python3-dev python3-pip python3-wstool protobuf-compiler dh-autoreconf -y
sudo pip3 install -U numpy protobuf scipy scikit-image ipython keras wrapt simplejson netaddr osrf-pycommon scipy pillow catkin_pkg rospkg opencv-python empy
sudo pip3 install -U tensorflow-gpu

mkdir -p ~/catkin_ws/src && cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/noetic --merge-devel
catkin config --cmake-args -DCMAKE_CXX_STANDARD=14 -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes
wstool init src

cd src
git clone ssh://git@chLi:30001/ustc/voxblox-plusplus.git
wstool merge -t . voxblox-plusplus/voxblox-plusplus_https.rosinstall
wstool update

sed -i -e 's/CV_BGR2GRAY/cv::COLOR_BGR2GRAY/g' -e 's/CV_CHAIN_APPROX_NONE/cv::CHAIN_APPROX_NONE/g' -e 's/CV_FILLED/cv::FILLED/g' depth_segmentation/depth_segmentation/src/depth_segmentation.cpp

