su
sudo cat /etc/sh
sudo cat /etc/shadow
sudo apt-get update
conda activate ros
roscore
cd catkin_ws/
catkin_make
source devel/setup.bash 
rostopic list
rostopic echo /camera/rgb/image_raw 
sudo apt install cheese
cheese
python3 --version
pip
sudo apt install python3-pip
pip
wget https://repo.anaconda.com/archive/Anaconda3-2023.03-Linux-x86_64.sh
chmod +x Anaconda3-2023.03-Linux-x86_64.sh
./Anaconda3-2023.03-Linux-x86_64.sh
source ~/.bashrc
conda --version
conda create -n ros python=3.8
conda activate ros
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
conda activate ros
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
roscore
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
pip install empy
catkin_make
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
catkin_make
pip install catkin_pkg
pip install rospkg
catkin_make
source devel/setup.bash 
conda deactivate
cd ..
conda deactivate
conda activate ros
rostopic list
roslaunch realsense2_camera rs_camera.launch
rostopic echo /camera/color/image_raw
conda activate ros
rostopic list
rqt_graph
cheese
rviz
rostopic list
rqt_graph
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
realsense-viewer
cd ~/catkin_ws/src
git clone https://github.com/pal-robotics/ddynamic_reconfigur
git clone https://github.com/pal-robotics/ddynamic_reconfigure
cd ..
catkin_make
cd ~/catkin_ws/src
git clone https://github.com/IntelRealSense/realsense-ros
cd ..
catkin_make
catkin_make_isolated
catkin_make_isolated --pkg realsense2_camera_msgs
cd ~/catkin_ws/src
mkdir -p ~/non_catkin_ws/src
mv realsense2_camera_msgs realsense2_description realsense2_camera ~/non_catkin_ws/src/
ls -al
rm -rf realsense-ros/
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
cd ..
cd src
catkin_init_workspace
cd ..
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
conda activate ros
roslaunch realsense2_camera rs_camera.launch
rostopic
rostopic list
roslaunch realsense2_camera rs_camera.launch
catkin_make
source devel/setup.bash 
roslaunch realsense2_camera rs_camera.launch
ls /dev/video*
roslaunch realsense2_camera rs_camera.launch
sudo apt-get update
sudo apt-get install ros-noetic-vision-msgs
roslaunch realsense2_camera rs_camera.launch
catkin_make
source devel/setup.bash 
roslaunch realsense2_camera rs_camera.launchcatc
roslaunch realsense2_camera rs_camera.launch
cd ~/humandetection_ros/
git pull
git add .
git status
git commit -m "edit pkg name"
git push
git config --global user.name "zihos"
git config --global user.name
git push
git config user.name
git config --global user.email "jiho8345@naver.com"
git push
git remote add origin https://github.com/zihos/humandetection_ros.git
git push
conda activate ros
python -c "from ultralytics import YOLO; print('Ultralytics YOLO loaded successfully')"
cd catkin_ws/
catkin_make
source devel/setup.bash 
rostopic list
rostopic echo /detections 
catkin_make
source devel/setup.bash 
rostopic list
rostopic echo /detections
rostopic list
rostopic echo /detections_3d 
rostopic echo /detections
rostopic echo /detections_3d 
rostopic list
rostopic echo /visualization_BEV
rostopic list
rostopic echo /detections_3d 
rostopic echo /detections
rostopic list
rostopic echo /detections
rostopic
rostopic list
rostopic echo /camera/color/image_raw 
git clone https://github.com/zihos/humandetection_ros.git
conda activate ros
sudo apt update
sudo apt install ros-noetic-vision-opencv
pip install opencv-python-headless
pip install numpy
pip install rospkg catkin_pkg
cd ~/catkin_ws/
source devel/setup.bash
catkin_create_pkg my_pkg rospy std_msgs sensor_msgs cv_bridge message_generation
cd src
catkin_create_pkg my_pkg rospy std_msgs sensor_msgs cv_bridge message_generation
cd ~/catkin_ws
catkin_make
source devel/setup.bash
cp -r /home/zio/humandetection_ros/my_pkg/data /home/zio/catkin_ws/src/my_pkg
cp -r /home/zio/humandetection_ros/my_pkg/msg /home/zio/catkin_ws/src/my_pkg
cp -r /home/zio/humandetection_ros/my_pkg/launch/ /home/zio/catkin_ws/src/my_pkg
catkin_make
pip uninstall empy
pip install empy
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws
catkin_make
pip uninstall empy
pip install empy=3.3.4
pip install empy==3.3.4
catkin_make
source devel/setup.bash 
cd /home/zio/catkin_ws/src/my_pkg/src
ls -al
cp /home/zio/humandetection_ros/my_pkg/src/video_publisher.py /home/zio/catkin_ws/src/my_pkg/src
ls -al
cd ../../..
rosrun my_pkg video_publisher.py 
catkin_make
source devel/setup.bash 
rosrun my_pkg video_publisher.py 
cd /home/zio/catkin_ws/src/my_pkg/data
ls -al
chmod +x pedestrian.mp4 
cd ../../..
cd /home/zio/catkin_ws/src/my_pkg/data
cd ../../..
rosrun my_pkg video_publisher.py 
catkin_make
source devel/setup.bash 
rosrun my_pkg video_publisher.py 
cd src/my_pkg/src/
chmod +x video_publisher.py 
ls -al
cd ../../..
roslaunch my_pkg yolov8_ros.launch 
python --version
conda install python=3.8.10
python --version
catkin_make
source devel/setup.bash 
roslaunch my_pkg yolov8_ros.launch 
pip install ultralytics
catkin_make
source devel/setup.bash 
roslaunch my_pkg yolov8_ros.launch 
catkin_make
source devel/setup.bash 
roslaunch my_pkg yolov8_ros.launch 
catkin_make
source devel/setup.bash 
roslaunch my_pkg yolov8_ros.launch 
catkin_make
source devel/setup.
source devel/setup.bash 
roslaunch my_pkg yolov8_ros.launch 
cp -r /home/zio/humandetection_ros/my_pkg_bev /home/zio/catkin_ws/src
catkin_make
roslaunch my_pkg_bev yolov8_ros.launch 
cd /home/zio/catkin_ws/src/my_pkg_bev/src
ls -al
chmod +x detection_3d_node.py 
ls -al
chmod +x tf_broadcaster.py 
chmod +x video_publisher.py 
chmod _x visualizer.py 
chmod +x visualizer.py 
ls -al
chmod +x yolov8_node.py 
ls -al
cd ../../..
roslaunch my_pkg_bev yolov8_ros.launch 
pip install tensorflow
catkin_make
pip show tensorflow
pip install tensorflow==2.10.0
catkin_make
source devel/setup.bash 
roslaunch my_pkg_bev yolov8_ros.launch 
catkin_make
source devel/setup.bash 
roslaunch my_pkg_bev yolov8_ros.launch 
catkin_make
source devel/setup.bash 
roslaunch my_pkg_bev yolov8_ros.launch 
catkin_make
source devel/setup.bash 
roslaunch my_pkg_bev yolov8_ros.launch 
catkin_make
source devel/setup.bash 
roslaunch my_pkg_bev yolov8_ros.launch 
catkin_make
source devel/setup.bash 
roslaunch my_pkg_bev yolov8_ros.launch 
catkin_make
source devel/setup.
source devel/setup.bash 
roslaunch my_pkg_bev yolov8_ros.launch 
catkin_make
source devel/setup.bash 
roslaunch my_pkg_bev yolov8_ros.launch 
sudo apt update
sudo apt install htop
htop
conda activate ros
rqt_graph
conda activate ros
rostopic list
catkin_make
source devel/setup.bash 
rostopic list
rostopic echo /detections 
rostopic echo /prediction_3d 
rostopic echo /detections 
rostopic echo /camera/color/image_raw
rostopic list
conda activate ros
cd catkin_ws/
catkin_make
source devel/setup.bash 
roslaunch my_pkg_bev yolov8_ros.launch 
catkin_make
source devel/setup.bash 
roslaunch my_pkg_bev yolov8_ros.launch 
conda activate ros
cd catkin_ws/
catkin_make
source devel/setup.bash 
roslaunch my_pkg_bev yolov8_ros.launch 
roslaunch realsense2_camera rs_camera.launch
conda activate ros
cd catkin_ws/
cat /proc/cpuinfo
conda activate ros
cd catkin_ws/
catkin_make
source devel/setup.bash 
roslaunch realsense2_camera rs_camera.launch
conda activate ros
catkin_make
source devel/setup.bash 
roslaunch my_pkg_bev yolov8_ros.launch 
catkin_make
source devel/setup.bash 
roslaunch my_pkg_bev yolov8_ros.launch 
catkin_make
source devel/setup.bash 
roslaunch my_pkg_bev yolov8_ros.launch 
conda activate ros
rqt_traph
rqt_graph
rostopic list
rostopic echo /detections 
rostopic echo /prediction_3d 
catkin_make
rostopic list
rostopic echo /detections
rostopic echo /pre
rostopic echo /prediction_3d 
cd humandetection_ros/
git add .
git status
git commit -m "add rosbag data"
git push
scp -P 9040 /home/zio/humandetection_ros/rosbag/2024-05-31-15-49-41.bag zio@210.94.179.18:/home/public/zio/humandetection_ros
cheese
scp -P 9040 /home/zio/humandetection_ros/rosbag/2024-05-31-15-49-41.bag zio@210.94.179.18:/home/public/zio/humandetection_ros
cd ../catkin_ws/
roslaunch my_pkg_bev yolov8_ros.launch 
conda activate ros
roslaunch my_pkg_bev yolov8_ros.launch 
catkin_make
source devel/setup.bash 
roslaunch my_pkg_bev yolov8_ros.launch 
conda activate ros
cd catkin_ws/src/my_pkg_bev/src/
ls -al
chmod +x bbox3d_utils.py 
chmod +x rosbag_check.py 
cheese
htop
conda activate ros
catkin_make
source devel/setup.bash 
roslaunch my_pkg_bev yolov8_ros.launch 
catkin_make
source devel/setup.bash 
roslaunch my_pkg_bev yolov8_ros.launch 
catkin_make
source devel/setup.bash 
roslaunch my_pkg_bev yolov8_ros.launch 
catkin_make
source devel/setup.bash 
roslaunch my_pkg_bev yolov8_ros.launch 
catkin_make
roslaunch my_pkg_bev yolov8_ros.launch 
catkin_make
source devel/setup.bash 
roslaunch my_pkg_bev yolov8_ros.launch 
rosbag record -a -O my_robot_data.bag
rosbag record /camera/color/image_raw /camera/depth/image_rect_raw
rosbag play 2024-05-31-15-27-46.bag
rosbag record /camera/color/image_raw /camera/depth/image_rect_raw trial1.bag
rosbag record /camera/color/image_raw /camera/depth/image_rect_raw
rosbag play 2024-05-31-15-49-41.bag
rosbag record /camera/color/image_raw /camera/depth/image_rect_raw
rosbag play 202024-05-31-16-16-51.bag
rosbag play 2024-05-31-16-16-51.bag
roslaunch my_pkg yolov8_ros.launch 
roslaunch my_pkg_bev yolov8_ros.launch 
catkin_make
source devel/setup.bash 
roslaunch my_pkg_bev yolov8_ros.launch 
catkin_make
source devel/setup.bash 
roslaunch my_pkg_bev yolov8_ros.launch 
conda activate ros
catkin_make
source devel/setup.bash 
rostopic list
rostopic echo /detections 
rostopic list
rostopic echo /detections 
rostopic echo /prediction_3d 
rostopic list
catkin_make
soure devel/setup.bash 
source devel/setup.bash 
rosrun my_pkg_bev rosbag_check.py
rostopic list
rosrun my_pkg_bev rosbag_check.py
mv /home/zio/catkin_ws/2024-05-31-15-49-41.bag /home/zio/humandetection_ros/rosbag
rosrun my_pkg_bev rosbag_check.py
rostopic list
rostopic echo /detections 
cd src/my_pkg_bev/src/
ls -al
chmod +x rgbd_distance_calculator.py 
cd ../../..
rostopic list
conda activate ros
cd catkin_ws/
ls -al
catkin_make
source devel/setup.bash 
roslaunch my_pkg_bev yolov8_ros.launch 
catkin_make
source devel/setup.bash 
roslaunch realsense2_camera rs_camera.launch
roscore
roslaunch realsense2_camera rs_camera.launch
roscore
roslaunch realsense2_camera rs_camera.launch
roscore
roslaunch realsense2_camera rs_camera.launch
htop
sudo apt-get install sysstat
mpstat 1 5 
conda activate ros
cd catkin_ws/
catkin_make
source devel/setup.bash 
roslaunch realsense2_camera rs_camera.launch
cd src/rgbd/
ls -al
scp -r -P 9040 zio@210.94.179.18:/home/public/zio/humandetection_ros/final_code_integration /home/zio/catkin_ws/src/rgbd
ls -al
cd final_code_integration/
ls -al
rm -rf .DS_Store 
ls -al
rm quantized_model_openvino.zip 
ls -al
rm yolov8n_openvino_model.zip 
ls -al
rostopic list
rostopic echo /camera/color/
rostopic echo /camera/color/image_raw
rostopic echo /camera/color/image_raw 
conda activate ros
catkin_make
source devel/setup.bash 
rosbag play --clock src/my_pkg/data/2024-05-31-16-16-51.bag 1 -r 0.1
rosbag play --clock src/daye/data/2024-05-31-16-16-51.bag 1 -r 0.1
ls -al
rosbag play --clock src/daye/data/2024-05-31-16-16-51.bag 1 -r 0.1
rosbag play --clock -r 0.1 src/daye/data/2024-05-31-16-16-51.bag
rostopic list
rostopic echo /yolov8/detections 
rostopic list
rostopic echo /camera/color/image_raw 
rostopic echo /yolov8/detections 
rosparam get /camera/depth_module/profile
rosparam list
rosparam /camera/realsense2_camera/color_width
rosparam get /camera/realsense2_camera/color_width
rostopic echo /camera/depth/image_rect_raw
cd /home/zio/humandetection_ros
git add .
git status
git commit -m "add rgbd distance"
git push
git log
git reset HEAD~2
git status
git add .
git status
git commit -m "add rgbd distance"
git push
conda activate ros
cd /home/zio/catkin_ws/src/rgbd
python camera_intrinsic.py 
python test.py 
pip install pyrealsense2
python test.py 
roscore
python camera_extrinsic.py 
python 2d_3d.py 
rostopic list
rostopic hz /camera/color/image_raw
rostopic list
rostopic echo /detections 
conda activate ros
catkin_make
source devel/setup.bash 
rostopic list
rostopic echo /detections 
rostopic hz /detections 
cd ../humandetection_ros/
git add /
git add .
git status
git commit -m "update rgbd"
git push
git pull
ls -al
cp -r daye /home/zio/catkin_ws/src
git pull
cp -rf daye/src/ /home/zio/catkin_ws/src/daye
scp -P 9040 zio@210.94.179.18:/home/public/zio/2024-05-31-16-16-51.bag /home/zio/catkin_ws/src/daye/data
/home/zio/catkin_ws/src/daye
cd /home/zio/catkin_ws/src/daye
ls -al
cd data/
ls -al
chmod +x 2024-05-31-16-16-51.bag 
ls -al
cd ../src/
ls -al
chmod +x rgbd_distance_calculator.py 
chmod +x video_publisher.py 
chmod +x vis
chmod +x visualizer.py 
ls -al
cd ../../..
roslaunch realsense2_camera rs_camera.launch json_file_path:=/home/zio/catkin_ws/src/HighAccuracyPreset.jso
roslaunch realsense2_camera rs_camera.launch depth_width:=1280 depth_height:=720 color_width:=1280 color_height:=720 json_file_path:=/home/zio/catkin_ws/src/HighAccuracyPreset.json
roslaunch realsense2_camera test.launch 
cd src/realsense-ros/realsense2_camera/launch/
chmod +x test.launch 
cd ../../../..
chmod +x test.launch 
roslaunch realsense2_camera test.launch 
cd src/realsense-ros/realsense2_camera/
find . -type f -exec chmod +x {} \;
cd ../../..
roslaunch realsense2_camera test.launch 
rosrun realsense2_camera test.launch 
roslaunch realsense2_camera rs_camera.launch depth_width:=1280 depth_height:=720 color_width:=1280 color_height:=720 json_file_path:=/home/zio/catkin_ws/src/HighAccuracyPreset.json
roslaunch realsense2_camera rs_camera.launch depth_width:=640 depth_height:=480 color_width:=640 color_height:=480 json_file_path:=/home/zio/catkin_ws/src/HighAccuracyPreset.json
catkin_make
source devel/setup.bash 
roslaunch realsense2_camera rs_camera.launch depth_width:=640 depth_height:=480 color_width:=640 color_height:=480 json_file_path:=/home/zio/catkin_ws/src/HighAccuracyPreset.json
catkin_make
source devel/setup.bash 
roslaunch realsense2_camera rs_camera.launch depth_width:=640 depth_height:=480 color_width:=640 color_height:=480 json_file_path:=/home/zio/catkin_ws/src/HighAccuracyPreset.json
catkin_make
source devel/setup.bash 
roslaunch realsense2_camera rs_camera.launch depth_width:=640 depth_height:=480 color_width:=640 color_height:=480 json_file_path:=/home/zio/catkin_ws/src/HighAccuracyPreset.json
conda activate ros
cd catkin_ws/
catkin_make
source devel/setup.bash 
roslaunch my_pkg_bev yolov8_ros.launch 
cp /home/zio/catkin_ws/src/my_pkg_bev/src/rgbd_distance_calculator.py /home/zio/humandetection_ros
roslaunch my_pkg_bev yolov8_ros.launch 
cd src/
catkin_create_pkg zio rospy cv_bridge sensor_msgs std_msgs
cd ..
catkin_make
source devel/setup.bash 
catkin_make
source devel/setup.
source devel/setup.bash 
roslaunch my_pkg_bev yolov8_ros.launch 
cp /home/zio/catkin_ws/src/my_pkg_bev/src/rgbd_distance_calculator.py /home/zio/humandetection_ros
roslaunch my_pkg_bev yolov8_ros.launch 
ls -al
catkin_make
source devel/setup.bash 
roslaunch daye yolov8_ros.launch real_time:=True is_bev_plot:=True is_img_w_3d:=False is_marker:=False
cd src/daye/
chmod +x src/
cd src/
ls -al
chmod +x detection_3d_node.py 
chmod +x new_visualizer.py 
chmod +x yolov8_node.py 
chmod +x tf_broadcaster.py 
cd ../../..
roslaunch daye yolov8_ros.launch real_time:=True is_bev_plot:=True is_img_w_3d:=False is_marker:=False
catkin_make
source devel/setup.bash 
roslaunch daye yolov8_ros.launch real_time:=True is_bev_plot:=True is_img_w_3d:=False is_marker:=False
catkin_make
source devel/setup.bash 
roslaunch daye yolov8_ros.launch real_time:=True is_bev_plot:=True is_img_w_3d:=False is_marker:=False
roscore
cd src/
catkin_create_pkg rgbd rospy std_msgs sensor_msgs cv_bridge message_generation
cd ..
cd src/rgbd/src/
ls -al
chmod +x rgbd_detection.py 
ls -al
cd ../../..
catkin_make
source devel/setup.bash 
roslaunch rgbd yolov8_ros.launch 
catkin_make
source devel
source devel/setup.bash 
roslaunch rgbd yolov8_ros.launch 
catkin_make
source devel/setup.bash 
roslaunch rgbd yolov8_ros.launch 
catkin_make
source devel/setup.bash 
roslaunch rgbd yolov8_ros.launch 
catkin_make
source devel/setup.bash 
roslaunch rgbd yolov8_ros.launch 
catkin_make
source devel/setup.bash 
roslaunch rgbd yolov8_ros.launch 
catkin_make
source devel/setup.bash 
roslaunch rgbd yolov8_ros.launch 
catkin_make
source devel/setup.bash 
roslaunch rgbd yolov8_ros.launch 
catkin_make
source devel/setup.bash 
roslaunch rgbd yolov8_ros.launch 
conda activate ros
catkin_make
source devel/setup.bash 
roslaunch realsense2_camera rs_camera.launch depth_width:=640 depth_height:=480 color_width:=640 color_height:=480 json_file_path:=/home/zio/catkin_ws/src/HighAccuracyPreset.json
clear
roslaunch realsense2_camera rs_camera.launch depth_width:=640 depth_height:=480 color_width:=640 color_height:=480 json_file_path:=/home/zio/catkin_ws/src/HighAccuracyPreset.json
conda activate ros
cd catkin_ws/
roslaunch rgbd yolov8_ros.launch 
catkin_make
source devel/setup.bash 
roslaunch rgbd yolov8_ros.launch 
catkin_make
source devel/setup.bash 
roslaunch RGB my_launch.launch 
roslaunch RGB my_launch.launch real_time:=True
roslaunch RGB my_launch.launch 
roslaunch rgbd yolov8_ros.launch 
rosrun rgbd rgbd_detection.py 
roslaunch rgbd yolov8_ros.launch 
roslaunch RGB my_launch.launch 
catkin_make
source devel/setup.bash 
roslaunch RGB my_launch.launch 
roslaunch rgbd yolov8_ros.launch 
cd /home/zio/catkin_ws/src/rgbd/images
rm -rf *.jpg
cd ../../RGB/images/0612/
rm -rf *.jpg
cd humandetection_ros/
git add .
git status
git commit -m "add experiment results"
git push
python results/analysis.py 
cd results/
python analysis.py 
cd ..
git add .
git status
git commit -m "add experiment results"
git push
cd results/
python analysis.py 
ls -al
python analysis.py 
scp -r -P 9040 zio@210.94.179.18:/home/public/zio/humandetection_ros/RGB /home/zio/catkin_ws/src
scp -r -P 9040 /home/zio/catkin_ws/src/rgbd/images/forpresentation zio@210.94.178.18:/home/public/zio/humandetection_ros/distance_measure
scp -r -P 9040 /home/zio/catkin_ws/src/rgbd/images/forpresentation zio@210.94.179.18:/home/public/zio/humandetection_ros/distance_measure
scp -r -P 9040 /home/zio/catkin_ws/src/RGB/images/distance_measure zio@210.94.179.18:/home/public/zio/humandetection_ros/distance_measure/rgb
cd ..
python mstat.py 
ls -al
python mpstat.py 
ls -al
python mpstat.py 
mpstat 1 5
python mpstat.py 
conda activate ros
rostopic list
rostopic hz /yolov8/detections 
rostopic list
rostopic hz /yolov8/detections 
python mpstat.py 
rostopic hz /yolov8/detections 
python mpstat.py 
rostopic hz /yolov8/detections 
python mpstat.py 
rostopic list
cd catkin_ws/
cd src/RGB/src/
ls -al
chmod +x mobilenet_node.py 
chmod +x yolov8_node.py 
cd ~
ls -al
python mpstat.py 
cd catkin_ws/
rostopic list
rqt_graph
rqt_graph all
rqt_graph --all
rqt_graph
cd src/rgbd/src/
chmod +x subscriber.py 
cd ../../..
rqt_graph
rostopic list
rostopic echo /detections_3d
rostopic list
rostopic echo /detections_3d
rostopic list
rostopic echo /yolov8/detections 
mpstat 1 5 
roslaunch realsense2_camera rs_camera.launch
mpstat 1 5 
mpstat 1 20
lscpu
lsb_release -a
import numpy as np
import cv2
import glob
# 체스보드 패턴의 크기
chessboard_size = (9, 6)
# 체스보드 패턴의 각 코너에 대한 3D 좌표 생성
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
# 3D 점과 2D 점을 저장할 배열
objpoints = []  # 3D 점
imgpoints = []  # 2D 점
# 캘리브레이션을 위한 이미지 경로 설정
images = glob.glob('calibration_images/*.jpg')  # 'calibration_images' 폴더에 있는 모든 jpg 이미지 사용
# 각 이미지에 대해 처리
for fname in images:;     img = cv2.imread(fname)
cv2.destroyAllWindows()
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
# 결과 출력
print("Camera matrix : \n", mtx)
print("Distortion coefficients : \n", dist)
# 캘리브레이션 결과를 저장할 파일 이름 설정
calibration_file = 'calibration.yaml'
cv_file = cv2.FileStorage(calibration_file, cv2.FILE_STORAGE_WRITE)
cv_file.write("camera_matrix", mtx)
cv_file.write("dist_coeff", dist)
cv_file.release() print(f"Calibration file saved to {calibration_file}")
conda activate ros
cd src/rgbd/
ls -al
python camera_intrinsic.py 
cheese
conda activate ros
cd catkin_ws/
roslaunch realsense2_camera rs_camera.launch json_file_path:=/home/zio/catkin_ws/src/HighAccuracyPreset.json
cheese
roslaunch realsense2_camera rs_camera.launch json_file_path:=/home/zio/catkin_ws/src/HighAccuracyPreset.json
roslaunch realsense2_camera rs_camera.launch depth_width:=640 depth_height:=480 color_width:=640 color_height:=480 json_file_path:=/home/zio/catkin_ws/src/HighAccuracyPreset.json
