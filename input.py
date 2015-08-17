#author: Rommel
import sys

#get amount of rows from user
def getRow():
	print("Each row has 14 trees")
	print("Max amount of rows 10")
	
	#continue looping till int is entered
	while True:
		try:
			total_rows = int(input("Enter Amount of rows : "))
		except ValueError:
			#loop again invalid type
			print("input is not an integer please enter an integer")
			continue
		else:
			#check if input is a valid number
			if (total_rows < 1) or (total_rows > 10):
				#loop again invalid number
				print("Enter a number between 0 and 11 exclusive")
				continue
			else:
				break

	
	return total_rows


#main function
rows = getRow()
print (rows)