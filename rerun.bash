#get current pwd
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

#generate world
python3 input.py

#make package
cd ../..
catkin_make
source devel/setup.bash

#--------------------------------------------------------
#move generated inc files to world
cd $DIR

mv wood.inc world
mv orchard.inc world
mv weedLocation.inc world
mv walls.inc world
mv o1.world world

#----------------------------------------------------------
#move the goal files that the dynamic object read to locations

mv pickerLocations locations
mv tractorLocations locations
mv dogLocation locations
mv binArea locations

#-------------------------------------------------------------
#run nodes
cd $DIR
xterm -hold -e roscore&


xterm -hold -e rosrun team4_ros MainNode locations/&

cd $DIR/world
xterm -hold -e rosrun stage_ros stageros o1.world


