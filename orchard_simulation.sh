#!/bin/bash

#create workspace
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

dir=~/ros_WS
build=true

#nick how does this info get printed
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

#ask user if they wish to remove dir or specify a different 1
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

#create dir when it does not exist
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
#add a line to seperate what happens
echo ""
echo "-------------------------------------------------------------------"
echo "                     Moving Files Over                            "
echo "-------------------------------------------------------------------"
echo ""


#-----------------------------------------------------------------------------
# Move packages to workspace
cd $DIR
cp -r team4_ros $dir/src/se306-1
cp -r world $dir/src/se306-1

#copy directory that setups files
cp -r setup $dir/src/se306-1
cp -r info $dir/src/se306-1

# create dir need to store generated files
cd $dir/src/se306-1
mkdir locations


#config world
cd $dir/src/se306-1/setup
python3 input.py


if [ $build = true ]
then
    # Build packages
    cd $dir
    catkin_make
fi

source $dir/devel/setup.bash

#----------------------------------------------------------------------------
#move generated inc files to world
cd $dir/src/se306-1/setup

mv wood.inc ../world
mv orchard.inc ../world
mv weedLocation.inc ../world
mv walls.inc ../world
mv o1.world ../world


#move the goal files that the dynamic object read to locations

mv pickerLocations ../locations
mv tractorLocations ../locations
mv dogLocation ../locations
mv binArea ../locations



#reset info files
cd $dir/src/se306-1/setup
python3 createNewInfoFiles.py


#----------------------------------------------------------------------------
#execute

# Go to workspace and run roscore
cd $dir
xterm -geometry -0+0 -title "Roscore" -hold -e roscore&
#xdotool search --sync --name "^Roscore$" windowsize --usehints 25% 50%

# Wait for roscore to initialise
sleep 1

# Open robot nodes
cd $dir/src/se306-1
xterm -hold -e rosrun team4_ros MainNode locations/&
#xdotool search --sync --name "^Main Node$" windowsize --usehints 25% 30%

#open orchard debugger
cd $dir/src/se306-1/setup
xterm -hold -e python3 printOut.py&
#xdotool search --sync --name "^Orchar Debugger$" windowsize --usehints 25% 30%


# Open world file
cd $dir/src/se306-1/world
xterm -hold -e rosrun stage_ros stageros o1.world
#xdotool search --sync --name "^Stage Output$" windowsize 25% 50%

# Resize stage window
#xdotool search --sync --name "^World" windowsize --usehints 40% 100%

wait
