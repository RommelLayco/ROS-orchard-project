#author: Rommel
#subauthored : Anmol
import sys

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
	current_y = 0 
	current_x = 0 + r_spacing/2  - 0.5

	# for i from 0 to 6 included (7 elements)
	for picker in range(0,7):

		name = "\"picker" + str(count) + "\""
		
		picker = "picker( pose [ " 
		picker = picker + str(current_x) + " "
		picker = picker + str(current_y) + " "
		picker = picker + "0 0 ] name " + name + " )\n"
	

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
	f.write('include "carrier.inc"\n\n') #canopy model

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

def configwall(r_spacing, t_spacing):
	f = open('wall.inc', 'w')



	#include model to tree
	f.write('include "talltree.inc\n"')

		#do nothing for now 



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

	line = "tallTree( pose [ " 
	line = line + str(-5) + " "
	line = line + str(-5) + " "
	line = line + "0 0 ] )\n"

	
	
	#add left wall
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
	line = line + "pose [ -16.000 " + str(y) + " 0.000 0.000]\n"
	line = line + "bitmap \"driveway.png\"\n)\n"

	return line




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



print("Finished making environment")
