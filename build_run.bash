#!/bin/bash

#create workspace
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

dir=~/ros_WS
build=true

help() {
    # Print out usage info
    echo "Usage information:"
    echo "-h    Show this help"
    echo "-nb   Don't build anything"
    echo "-r    Don't prompt if directory exists; just remove it"
}

# Process arguments
for var in "$@"
do
   case $var in
       -nb)
          # Build
          build=false
          ;;
      -r)
          remove=true
          ;;
      -h)
          help
          exit 0
          ;;
      *)
          echo "$var is not a valid argument"
          help
          exit 1
          ;;
  esac
done


if [ -d $dir/src/se306-1 ] && [ $build = true ] && [ ! $remove ]
then
      read -p "Directory '$dir' already exists. Do you wish to remove it?" response
      case $response in
        [yY][eE][sS]|[yY])
            rm -r $dir
            mkdir -p $dir/src/se306-1
            ;;
        *)
            read -p "Specify an alternative directory: " dir
            mkdir -p $dir/src/se306-1
       esac
fi

if [ ! -d $dir ]
then
    mkdir -p $dir/src/se306-1
fi

if [ $remove ]
then
    rm -r $dir
    mkdir -p $dir/src/se306-1
fi

cd $dir/src
catkin_init_workspace


# Move packages to workspace
cd $DIR
cp -r team4_ros $dir/src/se306-1
cp -r world $dir/src/se306-1

if [ $build = true ]
then
    # Build packages
    cd $dir
    catkin_make
fi

source $dir/devel/setup.bash

# Go to workspace and run roscore
cd $dir
xterm -geometry -0+0 -title "Roscore" -hold -e roscore&
xdotool search --sync --name "^Roscore$" windowsize --usehints 25% 50%

# Wait for roscore to initialise
sleep 1

# Open robot nodes
cd $dir/src/se306-1/team4_ros
xterm -geometry +0+0 -title "Picker Node" -hold -e rosrun team4_ros AlphaWithSensorRobotNode&
xdotool search --sync --name "^Picker Node$" windowsize --usehints 25% 30%

xterm -geometry +0+350 -title "Dog Node" -hold -e rosrun team4_ros DogNode&
xdotool search --sync --name "^Dog Node$" windowsize --usehints 25% 30%

xterm -geometry +0-0 -title "Person Node" -hold -e rosrun team4_ros PersonNode&
xdotool search --sync --name "^Person Node$" windowsize --usehints 25% 30%

# Open world file
cd $dir/src/se306-1/world
xterm -geometry -0-0 -title "Stage Output" -hold -e rosrun stage_ros stageros newworld.world &
xdotool search --sync --name "^Stage Output$" windowsize 25% 50%

# Resize stage window
xdotool search --sync --name "^Stage$" windowsize --usehints 40% 100%

wait
