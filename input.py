#author: Rommel
#subauthored : Anmol
import sys
from random import randint

#get row spacing from user
def getRowSpacing():
	print("Each row has 14 trees")
	print("Amount of rows fixed at 7")
	print("-----------------------------")
	print("\n")
	
	#continue looping till a valid input is entered
	while True:
		try:
			print("Enter a number between 3.5 and 8 inclusive")
			row_spacing = float(input("Enter size row spacing in meters: "))
		except ValueError:
			#loop again invalid type
			print("Input is not a number, please try again")
			continue
		else:
			#check if input is a valid number
			if (row_spacing < 3.5) or (row_spacing > 8):
				#loop again invalid number
				print("Enter a number between 3.5 and 8 inclusive")
				continue
			else:
				break

	print("\n")
	
	return row_spacing

#get tree spacing from user
def getTreeSpacing():

	#continue looping till a valid input is entered
	while True:
		try:
			print("Enter a number between 5 and 8 inclusive")
			tree_spacing = float(input("Enter size of tree_spacing in meters: "))
		except ValueError:
			print("Input is not a number, please try again")
			continue
		else:
			if (tree_spacing < 5) or (tree_spacing > 8):
				print("Enter a number between 5 and 8 inclusive")
				continue
			else:
				break

	print("\n")

	return tree_spacing

#import graphic models and create instance file. This method just adds the headers into the orchard.inc
# and sets it up for creating the environment
def writeModels():
	f = open('orchard.inc','w')
	f.write('include "tree.inc"\n') #tree model
	f.write('include "wood.inc"\n') #canopy model
	f.write('\n')#leave a line space
	f.close()

# this method creates trees and adds them to the orchard.inc file. It also makes
# canopies and adds them to the orchard.inc file 
def createTrees(r_spacing, t_spacing):
	f = open('orchard.inc', 'a')

	#outer loop creates line of trees with row spacing
	current_x = 0
	current_y = 0
	for line in range(0,8):

		#inner loop create tree vertically with a tree spacing
		for tree in range(0,14):
			#create line to write to file
			line = "tree( pose [ " 
			line = line + str(current_x) + " "
			line = line + str(current_y) + " "
			line = line + "0 0 ] )\n"

			#append tree to file
			f.write(line)
			
			#increment y position
			current_y = current_y + t_spacing

		#increment x position 
		current_x = current_x + r_spacing
		current_y = 0

		#leave a line space to seperate trees
		f.write('\n')

	# set the current x and y back to 0 to ensure the canopies are created on top of the trees made above
	current_y = 0
	current_x = 0
		
	#canopy
	for line in range(0,8):

		#inner loop create tree vertically with a tree spacing
		for canopy in range(0,14):
			#create line to write to file
			line = "canopy( pose [ " 
			line = line + str(current_x) + " "
			line = line + str(current_y) + " "
			line = line + "0 0 ] )\n"

			#append tree to file
			f.write(line)
			
			#increment y position
			current_y = current_y + t_spacing

		#increment x position 
		current_x = current_x + r_spacing
		current_y = 0	

		#leave a line space to seperate trees
		f.write('\n')

	f.close()

# this method appends the model of the canopy based on the dimensions mentioned by the user.
# this is done before the orchard.inc is made so that the new model of canopy is used for the environment
def changeCanopyModel(t_spacing):
	f = open('wood.inc', 'a')

	#length of canopy wood
	length = t_spacing + 0.08

	model_line = "model\n(\n"
	model_line = model_line + "size [" + str(length) + " "
	model_line = model_line + "0.08 0.04 ]\n"
	model_line = model_line + "pose [0.0 0.0 1.9 0.0]\ncolor \"burlywood\"\n)"
	
	#write to file
	f.write(model_line)
	f.write('\n')

	#write the base to the file
	base = "# Base\n size [0 0 0]\ncolor \"green\"\nstack_children 0\ngui_nose 0\nobstacle_return 0\n)\n"
	f.write(base)

	f.close()

#check if need to append to world file to prevent repeated append
def needToAppend():
	f = open('wood.inc', 'r')

	lastline = f.readlines()[-1]
	#print(lastline)
	
	#do not append
	if(lastline == ")\n") or (lastline == ")"):	
		return False
	else:
		#append
		return True


# add a picker to the environment 
# the picker has to be between each of the 7 rows.
def addPicker(r_spacing):

# append to the world file 7 picker robots
# loop from 0 to 7 as there are 7 rows. take the row spacing and column spacing in to count

	f = open('o1.world', 'a')

	count = 1
	

	# set the current x and y back to 0 to ensure the canopies are created on top of the trees made above
	current_y = -2 
	current_x = 0 + r_spacing/2  - 0.5

	# for i from 0 to 6 included (7 elements)
	for picker in range(0,7):

		name = "\"picker" + str(count) + "\""
		
		picker = "picker( pose [ " 
		picker = picker + str(current_x) + " "
		picker = picker + str(current_y) + " "
		picker = picker + "0 90 ] name " + name + " )\n"
	

		#append tree to file
		f.write(picker)

		count = count + 1
			
		#increment y position
		current_x = current_x + r_spacing

	f.close()

def add_Instances_to_world():
	f = open('o1.world', 'w')

	#adding models to the world file

	f.write('include "orchard.inc"\n') #tree model
	f.write('include "picker.inc"\n') #canopy model
	f.write('include "person1.inc"\n') #tree model
	f.write('include "dog.inc"\n') #canopy model
	f.write('include "bin.inc"\n') #tree model
	f.write('include "wood.inc"\n') # change canopy base on size
	f.write('include "carrier.inc"\n') #canopy model
	f.write('include "walls.inc"\n') # add walls
	f.write('include "weedLocation.inc"\n\n')#weed model


	f.write('interval_sim 100\n\n')

	f.close()

def add_actor_pos():

	f = open('o1.world', 'a')

	#add actor postions to world file

	#actor position for picker? maybe
	f.write('define actor position\n')
	f.write('(\n')
	f.write('name "actor"\n')
	f.write('size [0.000 0.000 0.000]\n')
	f.write('drive "diff"\n')
	f.write('stack_children 0\n')
	f.write('gui_nose 0\n')
	f.write('obstacle_return 0\n')
	f.write(')\n\n')

	#add more actors here

def add_model(r_spacing, t_spacing):
	f = open('o1.world', 'a')

	r = r_spacing

	#add drive way
	line = addDriveway(t_spacing)
	f.write(line)
	f.write('\n')




#create walls for orchard add to orchard instance
def createWalls(r_spacing, t_spacing):
	f = open('walls.inc', 'w') #file that defines the positions of the walls

	#code currently spawns a tree
	# can loop round and make tall as wall but will have gap
	# if a wall will use code above to config else if just a series of tall tree
	#will loop around and build 

	#note add walls to "----- add_instances_to_world method"

	r = r_spacing		
	t = t_spacing
		
	f.write('include "tallTree.inc"\n\n')


	#add left wall
	current_x = -10
	current_y = -10

	#work out length
	f.write("#left wall")
	l = int(13 * t_spacing + 10)
	for i in range(0, l):
		line = "tallTree( pose [ " 
		line = line + str(current_x) + " "
		line = line + str(current_y) + " "
		line = line + "0 0 ] )\n"

		current_y = current_y + 1
	
		#add left wall
		f.write(line)

	#continue left wall after driveway
	current_y = 13 * t_spacing + 5
	for i in range(0, 6):
		line = "tallTree( pose [ " 
		line = line + str(current_x) + " "
		line = line + str(current_y) + " "
		line = line + "0 0 ] )\n"
		
		current_y = current_y + 1

		f.write(line)

	#leave a line to seprerate
	f.write('\n')
	
	f.write("#top wall")
	#create top wall
	current_y = 13 * t_spacing + 10	
	current_x = -10
	l = int(7 * r_spacing + 21)
	for i in range(0, l):
		line = "tallTree( pose [ " 
		line = line + str(current_x) + " "
		line = line + str(current_y) + " "
		line = line + "0 0 ] )\n"
		
		current_x = current_x + 1

		f.write(line)

	#leave a line to seprerate
	f.write('\n')
	
	f.write("#right wall")
	#create right wall
	current_y = -10
	current_x = 7 * r_spacing + 11
	l = int(13 * t_spacing + 21)
	for i in range(0, l):
		line = "tallTree( pose [ " 
		line = line + str(current_x) + " "
		line = line + str(current_y) + " "
		line = line + "0 0 ] )\n"
		
		current_y = current_y + 1

		f.write(line)

	
	#leave a line to seprerate
	f.write('\n')
	
	f.write("#bottom wall")
	#create bottom wall
	current_y = -10
	current_x = -10
	l = int(7 * r_spacing + 21)
	for i in range(0, l):
		line = "tallTree( pose [ " 
		line = line + str(current_x) + " "
		line = line + str(current_y) + " "
		line = line + "0 0 ] )\n"
		
		current_x = current_x + 1

		f.write(line)



	

	f.close()

def addDriveway(t_spacing):

	#furtherest tree y co-ordinate
	#13 trees 
	#subtracting 6 leaves the bottom of the bend 1 away from the last tree
	# 13 x position leaves the end of the drivway 6 away from the last tree
	y = (t_spacing * 13) - 6

	line = "#driveway\n"
	line = line + "model\n(\n"
	line = line + "size [20.000 20.000 0.100 0]\n"
	line = line + "pose [ -20.000 " + str(y) + " 0.000 0.000]\n"
	line = line + "bitmap \"driveway.png\"\n)\n"

	return line

def addWeeds(r_spacing, t_spacing):
	f = open('weedLocation.inc', 'w')
	f.write('include "weed.inc"\n\n')

	#need 1 weed per row
	for i in range(0,7):
		pre_x = int(r_spacing)
		pre_y = int(t_spacing)

		#pick a tree to put a weed next to
		tree = randint(0, 12)

		#choose how far x the weed should be placed from the tree
		#i is the row
		x = randint(0, pre_x)
		x = (i * r_spacing) + x + 0.1

		#choose how far y the weed should be placed from the tree
		y = randint(0, pre_y)
		y = (tree * t_spacing) + y + 0.1

		
		#add weeds in a bunch of 4 to file (2 x 2)
		for h in range(0,2):
			for v in range(0,2):
				line = "weed( pose [" + str(x) + " "
				line = line + str(y) + " "
				line = line + "0 0])\n"
				f.write(line)

				#change vertical position
				y = y + 0.1

			#change horizontal position
			x = x + 0.1

		#leave a line to sepertate bunch			
		f.write('\n')

	#close file after writing
	f.close()

def dogLocation(r_spacing, t_spacing):

	#pick a row
	row = randint(0,6)
	x = row * r_spacing

	#pick a tree in the row
	tree = randint(0,13)
	y = tree * t_spacing

	#save location into a file
	f = open('dogLocation', 'w') 
	f.write(str(x) + " " + str(y) + "\n")
	f.close()


def tractorLocations(r_spacing, t_spacing):
	f = open('tractorLocations', 'w')

	#coordinates for top left
	x = -3
	y = (13 * t_spacing) + 3
	f.write(str(x) + " " + str(y) + "\n")

	#coordinates for top right
	x = (7 * r_spacing) + 3
	y = (13 * t_spacing) + 3
	f.write(str(x) + " " + str(y) + "\n")

	#coordinates for bottom right
	x = (7 * r_spacing) + 3
	y = -3
	f.write(str(x) + " " + str(y) + "\n")

	#coordinates for bottom left
	x = -3
	y = -3
	f.write(str(x) + " " + str(y) + "\n")

def pickerLocations(r_spacing, t_spacing):
	f = open('pickerLocations', 'w')

	#start point
	current_x = 0 + r_spacing/2  - 0.5
	current_y = -2

	#destnation point
	#note that the picker moves in a straight line up hence the x co ordinate stays the same
	final_y = 13 * t_spacing + 2


	for i in range(0,7):
		line = str(current_x) + " " + str(current_y) + " "
		line = line + str(current_x) + " " + str(final_y) + "\n"
		f.write(line)

		#change x positions
		current_x = current_x + r_spacing


	f.close()

def binArea(r_spacing, t_spacing):
	f = open('binArea', 'w')

	x = 4 * r_spacing
	y = 13 * t_spacing + 8

	line = str(x) + " " + str(y) +"\n"
	f.write(line)
	f.close()




'''----------------------------- Essentially the  main of the class below here ------------------------------'''


#main function
r_spacing = getRowSpacing()
t_spacing = getTreeSpacing()

'''--------------------------------------- cretae instance files here ---------------------------------------------'''

#need before create trees as create tree depends on the file created by changeCanopy
writeModels()

#check if you have to append
if(needToAppend()):
	changeCanopyModel(t_spacing)
	

createTrees(r_spacing, t_spacing)
addWeeds(r_spacing, t_spacing)
createWalls(r_spacing, t_spacing);


''' --------------------------------------- create world file put methods here --------------------------------'''

#add instances first
add_Instances_to_world()

#add actor positions
add_actor_pos()

#add models
add_model(r_spacing, t_spacing)

#add robots here
#picker first
addPicker(r_spacing)

'''------------------------------------ Destination locations for actors to read -----------------------------------'''
dogLocation(r_spacing,t_spacing)
tractorLocations(r_spacing,t_spacing)
pickerLocations(r_spacing, t_spacing)
binArea(r_spacing, t_spacing)




print("Finished making environment")
