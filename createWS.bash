#create workspace
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
echo $DIR


mkdir -p ~/ros_WS/src/se306-1
cd ~/ros_WS/src/
catkin_init_workspace
cd ..
catkin_make
source devel/setup.bash


#move packages to workspace
cd $DIR
mv team4_ros ~/ros_WS/src/se306-1
mv world ~/ros_WS/src/se306-1

#make
cd ~/ros_WS
catkin_make

