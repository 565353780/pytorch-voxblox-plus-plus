sed -i -e 's/import os/\nimport sys\nimport os\n\ncurPath = os.path.abspath(os.path.dirname(__file__))\nrootPath = os.path.split(curPath)[0]\nsys.path.append(rootPath + \"\/src\/mask_rcnn_ros\")\n/g' mask_rcnn_ros/scripts/mask_rcnn_node.py
sed -i -e "s/import tensorflow as tf/\nimport tensorflow.compat.v1 as tf\ntf.disable_v2_behavior()\ndevices = tf.config.experimental.list_physical_devices('GPU')\ntf.config.experimental.set_memory_growth(devices[0], True)\n/g" mask_rcnn_ros/src/mask_rcnn_ros/model.py
sed -i -e "s/import keras.engine as KE/import keras.engine.topology as KE/g" mask_rcnn_ros/src/mask_rcnn_ros/model.py
sed -i -e "s/(s\[1\]/(-1/g" mask_rcnn_ros/src/mask_rcnn_ros/model.py
sed -i -e "s/probs.shape\[0\]/1000/g" mask_rcnn_ros/src/mask_rcnn_ros/model.py
sed -i -e "s/from keras.engine import topology/from tensorflow.python.keras.saving import hdf4_format/g" mask_rcnn_ros/src/mask_rcnn_ros/model.py
sed -i -e "s/topology.load/hdf5_format.load/g" mask_rcnn_ros/src/mask_rcnn_ros/model.py
sed -i -e "22a\from PIL import Image\ndef scipy_misc_imresize(arr, size, interp='bilinear', mode=None):\n\t# im = scipy.misc.toimage(arr, mode=mode) # im为PIL.Image.Image对象\n\tim = Image.fromarray(arr, mode=mode)\n\tts = type(size)\n\tif np.issubdtype(ts, np.signedinteger):\n\t\tpercent = size / 100.0\n\t\tsize = tuple((np.array(im.size)*percent).astype(int))\n\telif np.issubdtype(type(size), np.floating):\n\t\tsize = tuple((np.array(im.size)*size).astype(int))\n\telse:\n\t\tsize = (size[1], size[0])\n\tfunc = {'nearest': 0, 'lanczos': 1, 'bilinear': 2, 'bicubic': 3, 'cubic': 3}\n\timnew = im.resize(size, resample=func[interp]) # 调用PIL库中的resize函数\n\t# return scipy.misc.fromimage(imnew)\n\treturn np.array(imnew)\n" mask_rcnn_ros/src/mask_rcnn_ros/utils.py
sed -i -e "s/scipy.misc.imresize/scipy_misc_imresize/g" mask_rcnn_ros/src/mask_rcnn_ros/utils.py
sed -i -e 's/CV_BGR2GRAY/cv::COLOR_BGR2GRAY/g' -e 's/CV_CHAIN_APPROX_NONE/cv::CHAIN_APPROX_NONE/g' -e 's/CV_FILLED/cv::FILLED/g' depth_segmentation/depth_segmentation/src/depth_segmentation.cpp

sed -i -e "s/<depend>opencv_catkin<\/depend>//g" ./depth_segmentation/depth_segmentation/package.xml
sed -i -e "s/<depend>pcl_catkin<\/depend>//g" ./depth_segmentation/depth_segmentation/package.xml
