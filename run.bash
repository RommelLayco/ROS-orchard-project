#get current pwd
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

#make package
cd ../..
catkin_make
source devel/setup.bash

#----------------------------------------------------------------------------------
#move static objects to world file
cd $DIR
#copy wood instance to world file
cp wood.inc world

#move orchard instance to world file
mv orchard.inc world

#move weedlocation instance to world file
mv weedLocation.inc world

#move wall instance to world file
mv walls.inc world

#move orchard world to world file
mv o1.world world

#----------------------------------------------------------------
#move dynmanic object info
#move generate file locations to locations folder

#move picker 
mv pickerLocations locations

#move tractor 
mv tractorLocations locations

#move dog locations
mv dogLocation locations

#move bin Area
mv binArea locations

#-------------------------------------------------------------

#run nodes
cd $DIR
xterm -hold -e roscore&

cd team4_ros
#xterm -hold -e rosrun team4_ros PersonNode&
#xterm -hold -e rosrun team4_ros DogNode&
xterm -hold -e rosrun team4_ros bin_node&
xterm -hold -e rosrun team4_ros PickerNode&
xterm -hold -e rosrun team4_ros CarrierNode&
xterm -hold -e rosrun team4_ros CarrierNode1&
xterm -hold -e rosrun team4_ros bin_node_t2&
xterm -hold -e rosrun team4_ros master_node&


cd $DIR/world
xterm -hold -e rosrun stage_ros stageros newworld.world
