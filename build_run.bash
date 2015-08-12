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

make
cd ~/ros_WS
catkin_make

#go to dir and run ros core
cd ~/ros_WS
xterm -geometry -0+0 -title "Roscore" -hold -e roscore&
xdotool search --sync --name "^Roscore$" windowsize --usehints 25% 50%

# Wait for roscore to initialise
sleep 2 

#open robot nodes
cd ~/ros_WS/src/se306-1/team4_ros
xterm -geometry +0+0 -title "Picker Node" -hold -e rosrun team4_ros AlphaWithSensorRobotNode&
xdotool search --sync --name "^Picker Node$" windowsize --usehints 25% 30%

xterm -geometry +0+350 -title "Dog Node" -hold -e rosrun team4_ros DogNode&
xdotool search --sync --name "^Dog Node$" windowsize --usehints 25% 30%

xterm -geometry +0-0 -title "Person Node" -hold -e rosrun team4_ros PersonNode&
xdotool search --sync --name "^Person Node$" windowsize --usehints 25% 30%

#open world file
cd ~/ros_WS/src/se306-1/world
xterm -geometry -0-0 -title "Stage Output" -hold -e rosrun stage_ros stageros newworld.world &
xdotool search --sync --name "^Stage Output$" windowsize 25% 50%

# Resize stage window
xdotool search --sync --name "^Stage$" windowsize --usehints 50% 100%

wait
