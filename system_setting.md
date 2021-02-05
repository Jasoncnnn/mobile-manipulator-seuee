
#ubuntu16.04系统快速安装和配置流程
0 删除亚马逊软件
sudo apt-get remove unity-webapps-common
0 卸载办公软件
sudo apt-get remove libreoffice-common
1 Rime输入法
2 git 使用
3 点文件配置
sudo apt-get install git
sudo apt-get install zsh
sudo apt-get install curl

3 假死
sudo rm /var/cache/apt/archives/lock

安装vscode
https://www.jianshu.com/p/9387d192f377

4 虚拟环境
5 ROS kinectic 版本安装
6 配置github上项目环境

7 常挂机设置
禁止休眠
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
解除禁止休眠
sudo systemctl unmask sleep.target suspend.target hibernate.target hybrid-sleep.target

8 把cpu调整到性能模式
sudo apt-get install cpufrequtils
cpufreq-info
sudo cpufreq-set -g performance
--------------------
链接：http://www.piggysnow.com/archives/308

9 gazebo模型下载
#模型的加载需要连接国外网站，为了保证模型顺利加载，可以提前将模型文件下载并放置到本地路径
~/.gazebo/models
https://bitbucket.org/osrf/gazebo_models/downloads/
#gazebo启动加载慢
cd ~/.gazebo/
mkdir -p models
cd ~/.gazebo/models/
wget http://file.ncnynl.com/ros/gazebo_models.txt
wget -i gazebo_models.txt
ls model.tar.g* | xargs -n1 tar xzvf
--------------------------------------
原文链接：https://blog.csdn.net/yang_guo_/article/details/89393712




10 rosbag数据记录与回放

mkdir ~/bagfiles
cd ~/bagfiles
rosbag record -a
rosbag record /topic1
#确保识别的物体ID从1开始，远离门轴的二维码为1，门轴的二维码为2
rosbag record /object_1/cmd_vel
#运算门转角度数并发布(30 pfs)
#存放位置~/sensor_ws/src/beginner_tutorials/src
#tf.cpp表示追踪远离门轴的二维码
rosrun beginner_tutorials tf_with2ar



#回放
rosbag info <your bagfile>
rosbag play <your bagfile>


11 aubo+base替换
aubo_robot->aubo_description->meshes->aubo_i5->collison->pedestal.STL


12 ROS+二维码
sudo apt-get install ros-kinetic-ar-track-alvar
opt->ros->kinetic->share->ar_track_alvar
#古月居-ROS机器人开发实践358页
roslaunch usb_cam usb_cam-test.launch
roslaunch robot_vision ar_track_camera.launch
#aubo i5 二维码伺服 eye on hand
#ar marker ID 12
roslaunch usb_cam usb_cam-test.launch
roslaunch aubo_i5_moveit_config moveit_planning_execution.launch robot_ip:=127.0.0.1
cd ~/catkin_ws/src/aubo_robot/aubo_demo/scripts
rosrun aubo_demo aubo_i5_ar_track.py
---------------------------------
https://www.guyuehome.com/6873



13 手柄测试界面
sudo apt-get install jstest-gtk
jstest-gtk


14 Ubuntu下的串口助手cutecom
sudo apt-get install cutecom
sudo cutecom


15 安装rqt-controller-manager
sudo apt-get install ros-kinetic-rqt-controller-manager
rosrun rqt_controller_manager rqt_controller_manager
#报错qt_gui_main() found no plugin matching "rqt_controller_manager则执行如下
rm ~/.config/ros.org/rqt_gui.ini


16 双网卡+linux+内外网

route -n
#wlp8s0无线网卡/enp9s0有线网卡
#校园网
sudo route add default gw 10.192.0.1
------------------
原文链接：https://www.jianshu.com/p/f8b7f034f044



17 tf坐标变换
#将当前的坐标系转换关系打印到终端控制台
rosrun tf tf_monitor
#tf_monitor <source_frame> <target_target>:打印特定的坐标系关系
rosrun tf tf_monitor /map /base_link
#tf_echo <source_frame> <target_frame> ：把特定的坐标系之间的平移旋转关系，打印到终端控制台
rosrun tf tf_echo /map /base_link
#rosrun tf tf_echo kinect2_link object_10
#发布一个父坐标系到子坐标系的静态tf转换，偏移x/y/z (单位是m)，旋转是欧拉角 yaw/pitch/roll (单位是弧度 rad)，这里 yaw是关于Z轴的旋转，pitch是关于关于Y轴的旋转，roll是关于X轴的旋转。这里的周期period_in_ms，是这个tf的发布周期，设置为100ms（也就是10hz）是一个很好的值。
static_transform_publisher x y z yaw pitch roll frame_id child_frame_id period_in_ms
-------------------------------------------
原文链接：https://blog.csdn.net/qq_39779233/article/details/108215144



18 获取门转角数据
#文件路径~/sensor_ws/src/beginner_tutorials/src/tf.cpp
rosrun beginner_tutorials my_tf_listener 
rqt_plot
#查看话题其中linear.x=OP，linear.x=OQ，linear.z=OQP_angle
/object_2/cmd_vel/linear

==========================================================================

#Ubuntu16.04 安装Rime
sudo apt install ibus-rime
sudo apt install librime-data-pinyin-simp

系统设置"System setting"里的“Text entry”进行图形化界面“+”操作
System setting-> language support-> Keyboard input method system-> IBus
Ctrl~选择简体字

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
mkdir GitProject
mkdir 项目本地仓库
cd 项目本地仓库
git init
————————————————
原文链接：https://www.runoob.com/w3cnote/git-five-minutes-tutorial.html


#gitlab
ssh-keygen -t rsa -C "你的邮箱地址"
新生成的ssh key在(/c/Users/admin/.ssh/id_rsa)里面，第一个是私钥，第二个是公钥，把公钥复制到你的gitlab中
git checkout liu.chuande
————————————————
原文链接：https://www.jianshu.com/p/291f4e526f87

==========================================================================

#安装haoming点文件
cd Downloads
git clone https://github.com/haomingw/dotfiles.git

==========================================================================

#安装ROS Kinectic
software updates-> ubuntu software-> 前四项打勾
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
#
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
#如果遇到连接到keyserver的问题，可以在以上命令尝试替换hkp://pgp.mit.edu:80或hkp://keyserver.ubuntu.com:80
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654

sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
#error##打开hosts文件
sudo gedit /etc/hosts
#在文件末尾添加
151.101.84.133  raw.githubusercontent.com
#保存后退出再尝试

rosdep update

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
#安装catkin工具
sudo apt-get install ros-kinetic-catkin python-catkin-tools


#zsh切换为bash
chsh -s /bin/bash
#bash切换为zsh
chsh -s /bin/zsh



#工作空间

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

cd ~/catkin_ws/
catkin_make

==========================================================================
==========================================================================
#快捷与记录

1 shell终端快捷键
2 lazygit快捷键
2 我的工作空间存储
3 aubo-i5项目启动
4 moveit官方panda-robot项目启动
5 快速编译古月居ros_exploring
6 KinectV2在ROS kinectic中调用libfreenect2和iai_kinect2
7 rviz中机器人的运动规划demo
8 Meka-robot平台启动
==========================================================================
==========================================================================

#Linux的shell终端常用快捷键
https://zhuanlan.zhihu.com/p/29538650</br>
#lazygit快捷键
https://github.com/linlicro/blog/blob/master/tools/lazygit%20.md</br>

==========================================================================

#工作空间存储

#古月居随书代码+aubo-robot代码
catkin_ws
#moveit官方panda-robot代码
ws_moveit
#Meka-robot代码
meka_ws

==========================================================================

#aubo—robot编译
#安装流程
git clone https://github.com/lg609/aubo_robot.git
--------------------------------
原文链接：https://blog.csdn.net/cstone123/article/details/95608424?utm_medium=distribute.pc_aggpage_search_result.none-task-blog-2~all~first_rank_v2~rank_v25-6-95608424.nonecase&utm_term=aubo%E5%A6%82%E4%BD%95%E4%BD%BF%E7%94%A8&spm=1000.2123.3001.4430

#缺少moveit_visual_tools
sudo apt-get install ros-kinetic-moveit-visual-tools
#缺少industrial_msgs
sudo apt-get install ros-kinetic-industrial-core
—————————————
原文链接：https://blog.csdn.net/eye998/article/details/88900155
#下載功能包后需要更新
cd /user_name/catkin_workspace/src/aubo_robot/UpdateMoveitLib/Kinetic/
chmod +x Update.sh
sudo ./Update.sh
#Skipping virtual joint 'base_link' because its child frame 'base_link' does not match the URDF 'world'
#aubo_i5.srdf中的virtual_joint下的child_link改为world
—————————————
原文链接：https://blog.csdn.net/weixin_42268975/article/details/104955479


================================================

#ubuntu安装openrave的简单方法
https://github.com/crigroup/openrave-installation
./install-dependencies.sh
./install-osg.sh
./install-fcl.sh
./install-openrave.sh
--------------------------------------------
https://blog.csdn.net/weixin_40512647/article/details/105719908



#MoveIt!中的运动学插件ikfast的配置(以aubo_i5机械臂为例)
原文链接：https://blog.csdn.net/fanky_chan/article/details/102547859?utm_medium=distribute.pc_aggpage_search_result.none-task-blog-2~all~first_rank_v2~rank_v25-1-102547859.nonecase&utm_term=aubo%E5%A6%82%E4%BD%95%E4%BD%BF%E7%94%A8&spm=1000.2123.3001.4430
---------------
注意：实际步骤在生成cpp时根据openrave-robot.py "$MYROBOT_NAME".dae --info links中模型index数设置如下
export BASE_LINK="1"
export EEF_LINK="7"

这里要注意下在Setup Assistant中配置运动规划组时，Add a planning group called manipulator that names the kinematic chain between base_link and tool0。
这里的设置要与这里一致，参照上面的关节数据：
在aubo_i5.srdf中确定。
<group name="manipulator_i5">
        <chain base_link="base_link" tip_link="wrist3_Link" />
-------------------
https://blog.csdn.net/harrycomeon/article/details/95517129
生成cpp过程大约耗时15分钟

#实际安装时候在openware环节cmake .. -DOSG_DIR=/usr/local/lib64/报错
#转为如下安装openware步骤
https://github.com/crigroup/openrave-installation
#并参考如下步骤
https://blog.csdn.net/Huster_mse/article/details/108725451
#实际安装时候在openware环节cmake .. -DOSG_DIR=/usr/local/lib64/报错FCL


================================================


#aubo运行rviz和gezebo
source aubo_ws/devel/setup.zsh
roslaunch aubo_i5_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=127.0.0.1
source aubo_ws/devel/setup.zsh
roslaunch aubo_demo MoveGroupInterface_To_Kinetic.launch
roslaunch aubo_gazebo aubo_i5_gazebo_control.launch
—————————————
#超时报错解决：https://blog.csdn.net/sinat_38284212/article/details/101626539
#rivz不能完全加載卸载moveit重装
sudo apt-get remove ros-kinetic-moveit-*
sudo apt-get install ros-kinetic-moveit

—————————————
解决方法：https://www.shangmayuan.com/a/a0bd4f0fa1be44f190a5d74d.html
##roslaunch aubo_demo MoveGroupInterface_To_Kinetic.launch不能正常执行
#注释掉其workspace以及删除于不用的workspace中编译产生的buiid和devel文件夹删去



#aubo-robot 笔记本配置示教器
#配置can-linux-driver
sudo apt-get install libpopt-dev
tar -zxvf peak-linux-driver-8.1.tar.gz
cd peak-linux-driver-8.1
make clean
make net=NO_NETDEV_SUPPORT
sudo make install
————————————————
原文链接：https://blog.csdn.net/wuguangbin1230/article/details/78327788



#在gazebo中使用aubo
sudo apt-get install ros-kinetic-desktop-full
sudo apt-get install ros-kinetic-transmission-interface
sudo apt-get install ros-kinetic-gazebo-ros-control
sudo apt-get install ros-kinetic-joint-state-controller
sudo apt-get install ros-kinetic-effort-controllers
sudo apt-get install ros-kinetic-position-controllers
sudo apt-get install ros-kinetic-control-manager


#aubo二次开发控制柜与笔记本网线/wifi连接
网线IP  192.168.1.3（笔记本）
	192.168.1.4（aubo 控制柜）
wifi IP 192.168.1.5（笔记本）
	192.168.1.6（aubo 控制柜）
掩码   255.255.255.0
子网    192.168.1.1
存在问题是：wifi连接时机械臂速度约束不够，运行卡顿。


#aubo的C++控制examples调用
cd ~/aubo_ws/src/aubo_robot/AuboInterfaceExample/
g++ -o main main.cpp
./main

#aubo+linux+sdk+官方实例

#首先安装qt creator
https://blog.csdn.net/vitor_lxy/article/details/94215967
#qt项目windows移植至linux
qmake -project QT+=widgets
qmake
make

==========================================================================

#手柄控制aubo i5


#labtop ip=192.168.1.3
roslaunch aubo_i5_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=192.168.1.4
#planning->allow external comm.
#add pose->topic->joy
roslaunch aubo_i5_moveit_config joystick_control.launch dev:=/dev/input/js0

-----------------------------------


#手柄控制底盘
#
sudo apt install ros-kinetic-serial
cd ~/base_ws
rosrun serial_msgs serial_example_node1

#键盘控制手爪
#
rosrun handControl handControl_node
rosrun handControl talker Enable 1
rosrun handControl talker Open 1
#手爪控制GUI
chmod u+x run_Grip-Manager.sh
chmod u+x Grip-Manager
cd ~/Desktop/Grip-Manager-ubuntu-16.04-x64 
./run_Grip-Manager.sh



==========================================================================

#UR5安装包编译
git clone -b kinetic-devel https://github.com/ros-industrial/universal_robot.git
cd ~/catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro kinetic
catkin_make
# if there is any error, try
pip uninstall em
pip install empy

#ur_modern_driver替换ur_driver更新
git clone https://github.com/iron-ox/ur_modern_driver.git

roslaunch ur_gazebo ur5.launch limited:=true
roslaunch ur5_moveit_config moveit_rviz.launch config:=true
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true
roslaunch ur_bringup ur5_bringup.launch limited:=true robot_ip:=IP_OF_THE_ROBOT [reverse_port:=REVERSE_PORT]


#手柄控制UR5
roslaunch ur5_moveit_config demo.launch
#Motion planning->planning requeset->planning group->manipulator
#planning->allow external comm.
roslaunch ur5_moveit_config joystick_control.launch


#UR5视觉伺服抓取gazebo仿真
cd ur_ws/src
git clone https://github.com/lihuang3/ur5_ROS-Gazebo.git
cp ~/ur_ws/src/ur5_ROS-Gazebo/src/ur_description/ur5.urdf.xacro ~/ur_ws/src/universal_robot/ur_description/urdf/
cp ~/ur_ws/src/ur5_ROS-Gazebo/src/ur_description/common.gazebo.xacro ~/ur_ws/src/universal_robot/ur_description/urdf/
cp ~/ur_ws/src/ur5_ROS-Gazebo/blocks_poses.h ~/ur_ws/devel/include/ur5_notebook/
catkin_make
#ur5.urdf.xacro报错
#将transmission_hw_interface:=hardware_interface/PositionJointInterface添加到ur5.urdf.xacro中的xacro函数的参数申明中，即：
<xacro:macro name="ur5_robot" params="prefix joint_limited
shoulder_pan_lower_limit:=${-pi} shoulder_pan_upper_limit:=${pi}
shoulder_lift_lower_limit:=${-pi} shoulder_lift_upper_limit:=${pi}
elbow_joint_lower_limit:=${-pi} elbow_joint_upper_limit:=${pi}
wrist_1_lower_limit:=${-pi} wrist_1_upper_limit:=${pi}
wrist_2_lower_limit:=${-pi} wrist_2_upper_limit:=${pi}
wrist_3_lower_limit:=${-pi} wrist_3_upper_limit:=${pi}
transmission_hw_interface:=hardware_interface/PositionJointInterface">
#hw_interface="${transmission_hw_interface}"添加到在ur5.urdf.xacro中实例化时的<xacro:ur_arm_transmission prefix="${prefix}"/>中，即：<xacro:ur_arm_transmission prefix="${prefix}" hw_interface="${transmission_hw_interface}"/>
------------------
https://github.com/lihuang3/ur5_ROS-Gazebo/issues/13#issuecomment-607096940
----------------------------------
https://github.com/lihuang3/ur5_ROS-Gazebo



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

#古月居ros_exploring安装PROBOT_Anno-master
git clone https://github.com/ps-micro/PROBOT_Anno
#
roslaunch probot_anno_moveit_config demo.launch
chmod +x moveit_ik_demo.py
rosrun probot_anno_demo moveit_ik_demo.py
rosrun probot_anno_demo moveit_fk_demo.py
rosrun probot_anno_demo moveit_cartesian_demo.py _cartesian:=True
rosrun probot_anno_demo moveit_cartesian_demo.py _cartesian:=False
rosrun probot_anno_demo moveit_attached_object_demo.py
-----------------------------------
https://www.guyuehome.com/2889

==========================================================================

#KinectV2在ROS kinectic中调用libfreenect2和iai_kinect2

#安装libfreenect2 自身驱动
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
=============================================================================

#KinectV2 ROS标定

roscore
rosrun kinect2_bridge kinect2_bridge _fps_limit:=2
rosrun kinect2_calibration kinect2_calibration chess6x8x0.024 record color
#标定文件位于
~/sensor_ws/src/iai_kinect2/kinect2_bridge/data/008197364647

roslaunch kinect2_bridge kinect2_bridge.launch
rosrun kinect2_viewer kinect2_viewer
---------------------------------------
https://blog.csdn.net/weixin_39928309/article/details/83047473
https://www.cnblogs.com/sasasatori/p/11794499.html
https://github.com/code-iai/iai_kinect2/tree/master/kinect2_calibration


=============================================================================

#KinectV2在ROS中驱动
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
================================================================================

#实时octomap和点云

sudo apt-get install ros-kinetic-octomap
roslaunch kinect2_bridge kinect2_bridge.launch publish_tf:=true #最后一个参数必须要，否则发布的点云数据没有tf坐标系，在rviz中是无法查看到的
roslaunch kinect2_bridge octomap_display.launch 
#在Displays面板中添加PointCloud2和OccupancyGrid插件，将Global Options中的Fixed Frame设置成kinect2_link
#将PointCloud2的Topic设置成kinect2/qhd/points，将OccupancyGrid的Topic设置成/octomap_full即可
#此方法需要TF坐标转换
rosrun tf static_transform_publisher 0 0 0 0 0 0 camera_link kinect2_link 10
-------------------------
https://blog.csdn.net/l1216766050/article/details/88093792?utm_medium=distribute.pc_relevant.none-task-blog-title-2&spm=1001.2101.3001.4242

#config
roslaunch kinect2_bridge kinect2_bridge.launch publish_tf:=true #最后一个参数必须要，否则发布的点云数据没有tf坐标系，在rviz中是无法查看到的
roslaunch aubo_i5_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=127.0.0.1
rosrun tf static_transform_publisher 0 0 0 0 0 0 camera_link kinect2_link 10


===================================================================================

#物体检测和识别功能（ROS下find_object_2d）

#uvc carmera
cd ~/catkin_ws/src
git clone https://github.com/ktossell/camera_umd
cd ~/catkin_ws
#报错libv4l2.h
sudo apt-get install libv4l-dev

catkin_make
source devel/setup.bash
#
sudo apt-get install ros-kinetic-find-object-2d


cd ~/catkin_ws
git clone https://github.com/introlab/find-object src/find_object_2d
catkin_make

#UVC摄像头find_object_2d
roscore
roslaunch usb_cam usb_cam-test.launch
#修改sensor_ws/src/usb_cam/launch/video1
rostopic list
rosrun uvc_camera uvc_camera_node device:=/dev/video1
#rosrun find_object_2d find_object_2d image:=/usb_cam/image_raw  #usb_cam使用
rosrun find_object_2d find_object_2d image:=/image_raw   #uvc_camera使用
#查看物体位置
rosrun find_object_2d print_objects_detected
rostopic echo /objects

--------------------------
https://blog.csdn.net/zbr794866300/article/details/100538794
#
https://blog.csdn.net/weixin_44827364/article/details/104318553?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachin
eLearnPai2-5.control&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-5.control


#KinectV2+ROS下find_object_2d
roslaunch kinect2_bridge kinect2_bridge.launch
rostopic list
#rosrun find_object_2d find_object_2d image:=/camera/rgb/image_color
rosrun find_object_2d find_object_2d image:=/kinect2/hd/image_color

-------------------------------
#通过ROS控制真实机械臂(16) --- 视觉抓取之find_object实现物体识别
https://blog.csdn.net/qq_34935373/article/details/103757619
#一起做ROS-DEMO系列 (2):基于find_object_2d的目标匹配识别
https://zhuanlan.zhihu.com/p/71603204



#KinectV2+ROS下find_object_3d
roslaunch kinect2_bridge kinect2_bridge.launch publish_tf:=true
#roslaunch kinect2_bridge kinect2_bridge.launch
roslaunch find_object_2d find_object_3d_kinect2.launch
rosrun find_object_2d print_objects_detected image:=/camera/color/image_raw
------------------------
#通过ROS控制真实机械臂(16) --- 视觉抓取之finTUd_object实现物体识别
原文链接：https://blog.csdn.net/qq_34935373/article/details/103757619
------------------------
http://wiki.ros.org/find_object_2d


=================================================================================

#Ubuntu16.04测试usb摄像头
cd ~/sensor_ws/src
git clone https://github.com/bosch-ros-pkg/usb_cam.git usb_cam
catkin_make
#
roslaunch usb_cam usb_cam-test.launch

#修改video0是电脑自身摄像头，修改video1是外接摄像头
roscd usb_cam/launch
sudo vi usb_cam-test.launch

#摄像头校准
https://blog.csdn.net/qq_25241325/article/details/82705003?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-1.channel_param&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-1.channel_param

======================================================================================

#Ubuntu16.04安装kinectV2并获取骨骼数据
http://cvrlcode.ics.forth.gr/web_share/OpenNI/NITE_SDK/
下载NiTE-Linux-x64-2.2至sensor_ws/src/
cd ~/sensor_ws/src
cd NiTE-Linux-x64-2.2
chmod 777 ./*
sudo sh install.sh
cat NiTEDevEnvironment >> ~/.bashrc
#
cd Samples/Bin
vi OpenNI.ini
#添加Repository=/usr/lib/OpenNI2/Drivers到该文件最下方
#测试
cd sensor_ws/src/NiTE-Linux-x64-2.2/Samples/Bin
./UserViewer
==============================
https://www.guyuehome.com/14570

#骨骼数据如不是需要可以不安装openni2_tracker或者 Kinect2 tracker
https://blog.csdn.net/ndyj0829/article/details/88981230


#配置kinect2_tracker
cd sensor_ws/src/
git clone https://github.com/mcgi5sr2/kinect2_tracker.git
ln -s ~/sensor_ws/src/NiTE-Linux-x64-2.2/Samples/Bin/NiTE2/ ~/.ros/NiTE2
cd ~/sensor_ws/src/kinect2_tracker
source setup_nite.bash
gedit CmakeLists.txt
#修改CMakeList.txt文件：
#找到set（NITE2_DIR ）和set（NITE2_LIB ）两行，改为：
修改如下：
set(NITE2_DIR ~/sensor_ws/src/NiTE-Linux-x64-2.2/)
set(NITE2_LIB ~/sensor_ws/src/NiTE-Linux-x64-2.2/Redist/libNiTE2.so)
cd ~/sensor_ws/
catkin_make

#测试
roslaunch kinect2_tracker tracker.launch
rosrun rviz rviz


#安装Openni２
cd ~/libfreenect2/build
sudo apt-add-repository ppa:deb-rob/ros-trusty
sudo apt-get update
sudo apt-get install libopenni2-dev
sudo apt install openni2-utils
sudo make install-openni2
NiViewer2


#配置openni2_tracker
sudo git clone https://github.com/ros-drivers/openni2_tracker.git
cd openni2_tracker
#修改CMakeList.txt文件：
#找到HINTS和HINTS两行，改为：
HINTS /home/liu/sensor_ws/src/NiTE-Linux-x64-2.2/Include)
HINTS /home/liu/sensor_ws/src/NiTE-Linux-x64-2.2/Redist
cd ~/sensor_ws/
catkin_make --pkg openni2_tracker
#
#编译catkin_make --pkg openni2_tracker报错fatal error: ros/ros.h: No such file or directory
cd ~/sensor_ws/src/openni2_tracker
sudo gedit CMakeLists.txt 
#include_directories(${catkin_INCLUDEDIR} 修改为 include_directories(${catkin_INCLUDE_DIRS}
#添加find_package(orocos_kdl REQUIRED)

#测试
roslaunch kinect2_bridge kinect2_bridge.launch
cd ~/sensor_ws/src/NiTE-Linux-x64-2.2/Redist
rosrun openni2_tracker openni2_tracker

————————————————
原文链接：https://blog.csdn.net/weixin_43046653/article/details/89633617



=======================================================================================

#kinect空格键从B到D保存深度图
https://blog.csdn.net/sunbibei/article/details/51594824
#同时读取Kinect的彩色图和深度图像并以时间戳作为文件名保存
save_rgbd_from_kinect2 文件夹
首先修改其中cpp的存储路径
/home/chuande/recordData/RGBD，改为自己路径
编译catkin_make save_rgbd_from_kinect2


roslaunch kinect2_bridge kinect2_bridge.launch
rosrun save_rgbd_from_kinect2 save_rgbd_from_kinect2
数据存储路径
/home/chuande/recordData/RGBD
————————————————
原文链接：https://blog.csdn.net/crp997576280/article/details/88377871

======================================================================================

#使用Kinect2获取激光数据

快速启动 
roslaunch bringup kinect2_depthimage_to_laserscan_rviz_view.launch


catkin_ws/src
bringup depthimage_to_laserscan 两个文件夹编译时报错
#rosmake 报错“No Module named 'rospkg' ”解决方法
https://blog.csdn.net/qq_22945165/article/details/97408951

————————————————
原文链接：https://blog.csdn.net/Forrest_Z/article/details/56665631/
https://blog.csdn.net/uunubt/article/details/81200330?utm_medium=distribute.pc_relevant_t0.none-task-blog-BlogCommendFromMachineLearnPai2-1.channel_param&depth_1-utm_source=distribute.pc_relevant_t0.none-task-blog-BlogCommendFromMachineLearnPai2-1.channel_param

==========================================================================

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


==========================================================================

ROS下基于OpenNI2使用rgbd camera (ASUS Xtion PRO LIVE)

https://blog.csdn.net/qq_40738034/article/details/97136145

==========================================================================

#rviz中机器人的运动规划demo
#ROS Kinetic版本panda robot
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

#Meka-robot
git clone https://github.com/ahoarau/mekabot.git ~/mekabot
cd ~/mekabot
git submodule init
git submodule update
git submodule foreach git checkout master


==========================================================================
==========================================================================
#机器学习记录
1 yolov3-door-detection启动
2 

==========================================================================
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

#chrome截长屏
按f12-> ctrl+shift+p-> full

==========================================================================
#cascad
8XmhAh@ZTVRGrSU

==========================================================================
#zsh下ros和conda共存

zshrc.local中注释掉第10行
# added by Miniconda3 installer
prepend_path $HOME/miniconda3/bin



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





==========================================================================


#ROS 操作杆控制

ls /dev/input/
sudo jstest /dev/input/js0
roscore
rosrun joy joy_node
rostopic echo joy

————————————————
原文链接：https://www.guyuehome.com/256
https://blog.csdn.net/answerMack/article/details/89365769
https://blog.csdn.net/qq_41834780/article/details/100106144
https://blog.csdn.net/qq_34935373/article/details/95882217?utm_medium=distribute.pc_relevant.none-task-blog-utm_term-5&spm=1001.2101.3001.4242

https://blog.csdn.net/sujun3304/article/details/18791843?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-1.channel_param&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-1.channel_param

https://blog.csdn.net/sujun3304/article/details/18794077?utm_medium=distribute.pc_relevant.none-task-blog-searchFromBaidu-8.channel_param&depth_1-utm_source=distribute.pc_relevant.none-task-blog-searchFromBaidu-8.channel_param

https://blog.csdn.net/Abril_/article/details/105586812?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522160221004019195246608498%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=160221004019195246608498&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_v2~rank_v28_p-2-105586812.pc_first_rank_v2_rank_v28_p&utm_term=%E6%89%8B%E6%9F%84+%E6%9C%BA%E5%99%A8%E4%BA%BA&spm=1018.2118.3001.4187



==========================================================================

#ROS与SICK TIM激光雷达配置
 
原文链接：http://mario.is-programmer.com/posts/211255.html
https://blog.csdn.net/weixin_42454034/article/details/103218669
https://blog.csdn.net/ignoreyou/article/details/79488650

https://www.jianshu.com/p/5f485c54c3df

https://www.cnblogs.com/21207-iHome/p/7944220.html



==========================================================================

#航发底盘串口通讯CRC16为CRC-CCITT(XModem)
#python crcmod实现crc16算法
sudo apt-get install python-crcmod
算法见链接：https://www.pythonf.cn/read/156369#_10
--------------------------
https://cloud.tencent.com/developer/article/1570611
------------------------


AA 20 20 15 00 C4 21 0D
21C4为CRC码
#航发底盘串口通信RS232-USB需要兼容ubuntu系统的串口线，否则PC/ubuntu无法控制
#ROS与串口通信
https://github.com/threefruits/ros_serial_code
-----------------------------
https://blog.csdn.net/Forrest_Z/article/details/55002484?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-1.control&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-1.control


roscore //必须先运行
rosrun my_serial_node my_serial_node



ls -l /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB*

====================

https://tr-ros-tutorial.readthedocs.io/zh_CN/latest/_source/practice/2.3.%E8%AE%A9%E5%BA%95%E7%9B%98%E5%8A%A8%E8%B5%B7%E6%9D%A5%EF%BC%81%E5%B7%AE%E9%80%9F%E6%8E%A7%E5%88%B6%E6%A8%A1%E5%9E%8B%E5%8F%8A%E4%B8%B2%E5%8F%A3%E9%A9%B1%E5%8A%A8%E7%BC%96%E5%86%99.html

==========================================================================


#手柄+机械臂运动
#测试手柄
jstest /dev/input/js0
=======================
#手柄GUI检测软件
sudo apt-get install jstest-gtk
jstest-gtk

======================
#rivz打开后motion planning红色
sudo apt-get install ros-kinetic-moveit-ros-visualization

==========================================================================

#pr2
#pr2在ros indigo安全运行kinetic需要手动安装不可用sudo apt-get install ros-kinetic-pr2-*命令
#Ubuntu16.04下编译pr2机器人
#安装失败放弃
----------------------------
原文链接：https://github.com/gnoliyil/pr2_kinetic_packages

#pr2_moveit_config安装
git clone https://github.com/davetcoleman/pr2_moveit_config.git
cd ..
rosdep install --from-paths src --ignore-src --rosdistro kinetic
catkin_make
source ~/catkin_ws/devel/setup.zsh
-----------------------------
原文链接：https://blog.csdn.net/RCJ__88/article/details/80248994?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-1.add_param_isCf&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-1.add_param_isCf

roslaunch pr2_gazebo pr2_empty_world.launch
roslaunch pr2_teleop teleop_joystick.launch


==========================================================================


#摇杆控制Moveit+rivz中panda robot
roslaunch panda_moveit_config demo.launch
#添加Motion planning
#Motion planning->planning requeset->planning group->panda_arm
#planning->allow external comm.
roslaunch panda_moveit_config joystick_control.launch dev:=/dev/input/js0
#Add “Pose” to RViz Displays and subscribe to /joy_pose in order to see the output from joystick
-----------------------------
原文链接：https://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/joystick_control_teleoperation/joystick_control_teleoperation_tutorial.html


==========================================================================

#手柄控制小乌龟
#文件所在位置
aubo_ws/src/base_controller
//
roscore
rosrun joy joy_node
rosrun turtlesim turtlesim_node
rosrun base_controller joy11.py
-------------------
https://blog.csdn.net/weixin_42913571/article/details/103862119




#初始化moveit_commander接口
rosrun moveit_commander moveit_commander_cmdline.py
#开启命令行接口工具与move_group节点建立连接
use <group name>
#
current
rec c

==========================================================================
#linux微信
解压运行
tar xvf linux-x64.tar.gz
./electronic-chat
---------------------
原文链接：https://blog.csdn.net/lxy_2011/article/details/86741562









