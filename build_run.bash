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
cp -r team4_ros ~/ros_WS/src/se306-1
cp -r world ~/ros_WS/src/se306-1

#make
cd ~/ros_WS
catkin_make

#go to dir and run ros core
cd ~/ros_WS
xterm -title "Roscore" -hold -e roscore&

#open robot nodes
cd ~/ros_WS/src/se306-1/team4_ros
xterm -title "Picker Node" -hold -e rosrun team4_ros AlphaWithSensorRobotNode&

xterm -title "Dog Node" -hold -e rosrun team4_ros DogNode&

xterm -title "Person Node" -hold -e rosrun team4_ros PersonNode&

#open world file
cd ~/ros_WS/src/se306-1/world
xterm -title "Stage Output" -hold -e rosrun stage_ros stageros newworld.world
