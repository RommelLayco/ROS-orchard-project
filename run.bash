#get current pwd
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

#make package
cd ../..
catkin_make
source devel/setup.bash

#run nodes
cd $DIR
xterm -hold -e roscore&

cd team4_ros
xterm -hold -e rosrun team4_ros AlphaWithSensorRobotNode&

cd $DIR/world
xterm -hold -e rosrun stage_ros stageros newworld.world
