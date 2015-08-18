#author: Rommel
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

#import graphic models and create instance file
def writeModels():
	f = open('orchard.inc','w')
	f.write('include "tree.inc"\n') #tree model
	f.write('include "wood.inc"\n') #canopy model
	f.write('\n')#leave a line space
	f.close()

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

	for line in range(0,8):

		#inner loop create tree vertically with a tree spacing
		for canopy in range(0,14):
			#create line to write to file
			line = "wood( pose [ " 
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
	
	#do not append
	if(lastline == ")\n"):	
		return False
	else:
		#append
		return True






#main function
r_spacing = getRowSpacing()
t_spacing = getTreeSpacing()


writeModels()
#need before create trees as create tree depends on the file created by changeCanopy

#check if you have to append
if(needToAppend):
	changeCanopyModel(t_spacing)

createTrees(r_spacing, t_spacing)

print("Finished making environment")