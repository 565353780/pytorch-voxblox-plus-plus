# Pytorch Voxblox Plus Plus

## Download models
```bash
https://dl.fbaipublicfiles.com/detectron2/COCO-InstanceSegmentation/mask_rcnn_R_101_FPN_3x/138205316/model_final_a3ec72.pkl
https://gateway.infinitescript.com/?fileName=GRNet-ShapeNet.pth
```

and save them to
```bash
~/.ros/
```

## Build
```bash
mkdir -p ~/vpp_ws/src
cd ~/vpp_ws/src
git clone https://github.com/565353780/pytorch-voxblox-plus-plus.git
cd pytorch-voxblox-plus-plus
./setup.sh
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

## Enjoy it~

