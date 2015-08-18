#author: Rommel
import sys

#get row spacing from user
def getRowSpacing():
	print("Each row has 14 trees")
	print("Amount of rows fixed at 7")
	
	#continue looping till a valid input is entered
	while True:
		try:
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

	
	return row_spacing

#get tree spacing from user
def getTreeSpacing():

	#continue looping till a valid input is entered
	while True:
		try:
			tree_spacing = float(input("Enter size of tree_spacing in meters: "))
		except ValueError:
			print("Input is not a number, please try again")
			continue
		else:
			if (tree_spacing < 5) or (tree_spacing > 8):
				print("Enter a number between 4 and 8 inclusive")
				continue
			else:
				break


	return tree_spacing

#import graphic models and create instance file
def writeModels():
	f = open('orchard.inc','w')
	f.write('include "tree.inc"\n') #tree model
	f.write('include "canopy.inc"\n') #canopy model
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

		#leave a line space to seperate trees
		f.write('\n')

	f.close()




#main function
r_spacing = getRowSpacing()
t_spacing = getTreeSpacing()

t = str(r_spacing) + " ------sdfkdhf"

line = "tree( pose [ " 
line = line + str(r_spacing) + " "
line = line + str(t_spacing) + " "
line = line + "0 0 ] )"
			
print(line)

print (r_spacing)
print (t_spacing)

writeModels()
createTrees(r_spacing, t_spacing)
