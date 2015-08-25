#use this file if you only run the input.py

#get current pwd
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

#--------------------------------------------------------
#move generated inc files to world
cd $DIR

mv wood.inc ../world
mv orchard.inc ../world
mv weedLocation.inc ../world
mv walls.inc ../world
mv o1.world ../world

#----------------------------------------------------------
#move the goal files that the dynamic object read to locations

mv pickerLocations ../locations
mv tractorLocations ../locations
mv dogLocation ../locations
mv binArea ../locations
mv bigBinLocation ../locations
