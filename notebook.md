#cascad
8XmhAh@ZTVRGrSU

==========================================================================
#zsh下ros和conda共存

zshrc.local中注释掉第10行
# added by Miniconda3 installer
prepend_path $HOME/miniconda3/bin

==========================================================================

#工作空间存储

#古月居随书代码+aubo-robot代码
catkin_ws
#moveit官方panda-robot代码
ws_moveit
#Meka-robot代码
meka_ws

==========================================================================

#git使用
git clone http://git-scm.com/
git --version
git config --global user.name "<用户名>"
git config --global user.email "<电子邮件>"

#彩色显示
git config --global color.ui auto

#缩写co代表checkout
git config --global alias.co checkout
#win中使用Git Bash
git config --global core.quotepath off

#创建本地仓库
mkdir MyGitBox
cd MyGitBox
git init

原文链接：https://www.runoob.com/w3cnote/git-five-minutes-tutorial.html

#gitlab
ssh-keygen -t rsa -C "你的邮箱地址"
新生成的ssh key在(/c/Users/admin/.ssh/id_rsa)里面，第一个是私钥，第二个是公钥，把公钥复制到你的gitlab中
git checkout liu.chuande
————————————————
原文链接：https://www.jianshu.com/p/291f4e526f87


==========================================================================

#Meka-robot
git clone https://github.com/ahoarau/mekabot.git ~/mekabot
cd ~/mekabot
git submodule init
git submodule update
git submodule foreach git checkout master


==========================================================================
#aubo—robot编译

#缺少moveit_visual_tools
sudo apt-get install ros-kinetic-moveit-visual-tools
#缺少industrial_msgs
sudo apt-get install ros-kinetic-industrial-core
————————————————
原文链接：https://blog.csdn.net/eye998/article/details/88900155

cd /user_name/catkin_workspace/src/aubo_robot/UpdateMoveitLib/Kinetic/
chmod +x Update.sh
sudo ./Update.sh


#在gazebo中使用aubo
sudo apt-get install ros-kinetic-desktop-ful
sudo apt-get install ros-kinetic-transmission-interface
sudo apt-get install ros-kinetic-gazebo-ros-control
sudo apt-get install ros-kinetic-joint-state-controller
sudo apt-get install ros-kinetic-effort-controllers
sudo apt-get install ros-kinetic-position-controllers
sudo apt-get install ros-kinetic-control-manager

==========================================================================
#moveit官方panda-robot


rosdep update
sudo apt-get update
sudo apt-get dist-upgrade
sudo apt-get install ros-kinetic-catkin python-catkin-tools
sudo apt install ros-kinetic-moveit

mkdir -p ~/ws_moveit/src
cd ~/ws_moveit/src
git clone -b kinetic-devel https://github.com/ros-planning/moveit_tutorials.git

git clone -b kinetic-devel https://github.com/ros-planning/panda_moveit_config.git
cd ~/ws_moveit/src
rosdep install -y --from-paths . --ignore-src --rosdistro kinetic

cd ~/ws_moveit
catkin config --extend /opt/ros/kinetic
catkin build

source ~/ws_moveit/devel/setup.bash
echo 'source ~/ws_moveit/devel/setup.bash' >> ~/.bashrc

==========================================================================
#快速编译古月居ros_exploring
catkin_ws
#编译前需要安装的包
sudo apt-get install ros-kinetic-gazebo-ros-control
sudo apt-get install ros-kinetic-ecto
sudo apt install ros-kinetic-manipulation-msgs
sudo apt-get install ros-kinetic-slam-gmapping
sudo apt-get install ros-kinetic-moveit-ros-planning-interface
sudo apt-get install ros-kinetic-navigation

#讯飞语音库，可以把robot——perception中的robot_voice删除


#opencv
sudo apt-get install ros-kinetic-vision-opencv libopencv-dev python-opencv
#二维码功能
sudo apt-get install ros-kinetic-ar-track-alvar

#语音报错安装
#编译讯飞语音库的时候报错时安装
sudo cp ros_exploring/robot_perception/robot_voice/libs/x64/libmsc.so /usr/lib/libmsc.so
————————————————
原文链接：https://www.twblogs.net/a/5d54453fbd9eee5327fcfd8d?lang=zh-cn

缺少 python-gst 安装
sudo apt install python-gst0.10
缺少gconfaudiosrc安装
sudo apt-get install gstreamer0.10-gconf

==========================================================================
#安装ROS

software updates-> ubuntu software-> 前四项打勾
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential



#工作空间

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

cd ~/catkin_ws/
catkin_make


==========================================================================

#Ubuntu16.04 安装Rime
sudo apt install ibus-rime
sudo apt install librime-data-pinyin-simp

系统设置"System setting"里的“Text entry”进行图形化界面“+”操作
System setting-> language support-> Keyboard input method system-> IBus

==========================================================================

#安装haoming点文件
cd Downloads
git clone https://github.com/haomingw/dotfiles.git

==========================================================================
#KinectV2在ROS kinectic中调用libfreenect2和iai_kinect2

#安装libfreenect2 
cd ~/catkin_ws/src 
git clone https://github.com/OpenKinect/libfreenect2.git 
cd libfreenect2 
sudo apt-get install build-essential cmake pkg-config 
sudo apt-get install libusb-1.0-0-dev 
sudo apt-get install libturbojpeg libjpeg-turbo8-dev 
sudo apt-get install libglfw3-dev 
sudo apt-get install beignet-dev 
sudo apt-get install libopenni2-dev 
mkdir build && cd build 
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2 
make 
make install 
sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/ 
sudo apt-get install openni2-utils && sudo make install-openni2 && NiViewer2 


cd ~/catkin_ws/src/libfreenect2/build
./bin/Protonect
————————————————
原文链接：https://blog.csdn.net/zhangjiali12011/java/article/details/97112978


#安装iai_kinect2
cd ~/catkin_ws/src/
git clone https://github.com/code-iai/iai_kinect2.git
cd iai_kinect2
rosdep install -r --from-paths .
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE="Release"


roslaunch kinect2_bridge kinect2_bridge.launch
rosrun kinect2_viewer kinect2_viewer
————————————————
原文链接：https://blog.csdn.net/zhangjiali12011/java/article/details/97112978

==========================================================================

#rviz中机器人的运动规划demo
#ROS Kinetic版本
sudo apt-get install ros-kinetic-franka-description
cd ~/catkin_ws/src
git clone -b kinetic-devel https://github.com/ros-planning/moveit_tutorials.git
git clone -b kinetic-devel https://github.com/ros-planning/panda_moveit_config.git
cd ~/catkin_ws/
catkin_make

#ROS Melodic版本
sudo apt-get install ros-melodic-franka-description
cd ~/catkin_ws/src
git clone https://githelodicub.com/ros-planning/moveit_tutorials.git -b melodic-devel
git clone https://github.com/ros-planning/panda_moveit_config.git -b melodic-devel
cd ~/catkin_ws/
catkin_make

roslaunch panda_moveit_config demo.launch
rosrun moveit_tutorials move_group_python_interface_tutorial.py
————————————————
原文链接：https://www.guyuehome.com/7642


==========================================================================

#yolov3-door-detection启动
u cv37
pip install opencv-contrib-python
python
import cv2
cv2.__version__

python yolo_video.py --input videos/door_open_object_test.mp4 --output door_open_object_test.avi --yolo yolo-coco

#出现ImportError: /opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so: undefined symbol: PyCObject_Type
将以下代码放入yolo_video.py首行
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

==========================================================================

## Run the server and visualize the robot on Rviz (virtual installation)
```bash
# run the realtime server
m3rt_server_run 
# In another terminal :
# Launch roscore, robot description, robot state publisher, joint state publisher and rviz
roslaunch meka_description m3ens_viz.launch 
```

==========================================================================











#oh-my-Zsh：未找到Conda / Pip installs命令

vi~ / .zshrc 或 gedit~ / .zshrc

对于现场的Anaconda3，请# User configuration添加：

PATH="$HOME/anaconda3/bin:$PATH"

source ~/.zshrc

https://xbuba.com/questions/31615322







#unbuntu18.04安装openni2过程记录

步骤一：安装依赖

sudo apt-get install -y g++ python libusb-1.0-0-dev freeglut3-dev doxygen graphviz
sudo apt-get install libudev-dev
步骤二：源中寻找ｏｐｅｎｎｉ２

apt-cache search openni2
注：若没找到，则是源问题，更换源；

步骤三：安装

sudo apt-get install libopenni2-dev

sudo apt-get install ros-<rosdistro>-openni2-launch


步骤四：测试是否安装成功

pkg-config --modversion libopenni2
若结果显示版本号，则安装成功！

（迄今为止，最简单的安装）

使用openni2_launch

roscore

roslaunch openni2_launch openni2.launch

rosrun rviz rviz


add camera、pointcloud2 选择正确的topic就可以实时看到xtion返回的数据了





https://blog.csdn.net/qq_40738034/article/details/97135494

https://blog.csdn.net/chenxingwangzi/article/details/48825767



================================

ROS下基于OpenNI2使用rgbd camera (ASUS Xtion PRO LIVE)

https://blog.csdn.net/qq_40738034/article/details/97136145


