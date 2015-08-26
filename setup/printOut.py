#Author Rommel Layco

import os
import sys
from select import select

def Menu ():
	print("---------------------------------------------------------------")
	print("			Orchard Debugger		")
	print("---------------------------------------------------------------")
	print("please enter the node you want to check")


	#continue looping till a valid input is entered
	while True:
		try:
			char =  input("p = picker, c = carrier, h = human, a = animal, t = tractor, q = quit: ")
		except ValueError:
			#loop again invalid type
			print("Input is not a char")
			continue
		else:
			if char == 'p' or char == 'c' or char == 'h' or char == 'a' or char == 't' or char == 'q':
				break
			else :
				print("You did not select a valid char")
				print("Try again\n")
				print("Select a char from the following")
				
	
	return char


def loadFunction(char):
	if char == 'p':
		num = choosePicker()
		openPicker(num)
	elif char == 'c':
		num  = chooseCarrier()
		openCarrier(num)
	elif char == 'h':
		num = chooseHuman()
		openHuman(num)
	elif char == 'a':
		num = chooseAnimal()
		openAnimal(1)
	elif char == 't':
		num = chooseTractor()
		openTractor(1)
	else:
		pass

def clear():
	os.system('cls' if os.name =='nt' else 'clear')
		

#stop printing info of node when user press x
def waitToExit(filePath):


	
	#read continously till user enters x
	pre_last = ""
	while(True):			
		timeout = 2
		rlist, _, _ = select([sys.stdin], [], [], timeout)
				
		if rlist:
			stop = sys.stdin.readline()

			#stop reading and hence return to main menu
			if(stop == 'x\n'):
				break

			
		
		else:
			#read last line from file
			f = open(filePath, 'r')
			
			lastline = f.readlines()

			f.close()
			
			#only print if something new
			if pre_last != lastline:
				l = lastline[-1];
				print(l[:-1])
				pre_last = lastline
			
				
			

			
						


''' -------------------------------------- choose robot number -----------------------'''

def choosePicker():
	print('\n')
	print("---------------------------------------------------------------")
	print("			Picker Menu		")
	print("---------------------------------------------------------------")
	print("please enter the picker id number you want to check")


	#continue looping till a valid input is entered
	while True:
		try:
			num =  int(input("Choose from picker 1 - 7 inclusive: "))
		except ValueError:
			#loop again invalid type
			print("Input is not an intger")
			continue
		else:
			if num > 0  and num < 8:
				break
			else :
				print("You did not select a valid picker number")
				print("Try again\n")
				
				
	
	return num

def chooseCarrier():
	print('\n')
	print("----------------------------------------------------------------")
	print("			Carrier Menu		")
	print("----------------------------------------------------------------")
	print("please enter the carrier id number you want to check")


	#continue looping till a valid input is entered
	while True:
		try:
			num =  int(input("Choose from carrier 1 - 7 inclusive: "))
		except ValueError:
			#loop again invalid type
			print("Input is not an intger")
			continue
		else:
			if num > 0  and num < 10:
				break
			else :
				print("You did not select a valid picker number")
				print("Try again\n")
				
				
	
	return num

def chooseHuman():
	print('\n')
	print("----------------------------------------------------------------")
	print("			Human Menu		")
	print("----------------------------------------------------------------")
	print("please enter the human id number you want to check")


	#continue looping till a valid input is entered
	while True:
		try:
			num =  int(input("worker = 1, vistor = 2, person = 3: "))
		except ValueError:
			#loop again invalid type
			print("Input is not an intger")
			continue
		else:
			if num > 0  and num < 3:
				break
			else :
				print("You did not select a valid human id")
				print("Try again\n")
				
				
	
	return num

def chooseAnimal():
	print('\n')
	print("----------------------------------------------------------------")
	print("			Animal Menu		")
	print("----------------------------------------------------------------")
	print("There is only one animal, a dog please wait for information")

'''
	#continue looping till a valid input is entered
	while True:
		try:
			num =  int(input("dog = 1: "))
		except ValueError:
			#loop again invalid type
			print("Input is not an intger")
			continue
		else:
			if num > 0  and num < 3:
				break
			else :
				print("You did not select a valid animal")
				print("Try again\n")
				
				
	
	return num
	'''


def chooseTractor():
	print('\n')
	print("----------------------------------------------------------------")
	print("			Tractor Menu		")
	print("----------------------------------------------------------------")
	print("There is only 1 tractor please wait for information")

	num = 1

'''
	#continue looping till a valid input is entered
	while True:
		try:
			num =  int(input("Choose from carrier 1 - 10 inclusive: "))
		except ValueError:
			#loop again invalid type
			print("Input is not an intger")
			continue
		else:
			if num > 0  and num < 10:
				break
			else :
				print("You did not select a valid picker number")
				print("Try again\n")
'''




''' --------------------------- Read File -------------------------------------------------------------'''
def openPicker(num):

	print("\nPress x to return to main orchard menu")

	for i in range(1,8):
		if(i == num):
			filePath = '../info/picker/picker' + str(num) +'.txt'
			f = open(filePath, 'r')
			
			#read and print continously till user enters x
			waitToExit(filePath)
				

		elif (i == 8 and i != num):
			print("Could not find any info on picker " + str(num) )
		else :
			pass

def openCarrier(num):

	print("Press x to return to main orchard menu")

	for i in range(1,11):
		if(i == num):
			filePath = '../info/carrier/carrier' + str(num) +'.txt'
			
			#put wait to exit code here
			waitToExit(filePath)
			

		elif (i == 7 and i != num):
			print("Could not find any info on carrier " + str(num) )
		else :
			pass

def openHuman(num):

	print("Press x to return to main orchard menu")

	for i in range(1,3):
		if(i == num):
			filePath = '../info/human/human' + str(num) +'.txt'
			
			#put wait to exit code here
			waitToExit(filePath)
			

		elif (i == 3 and i != num):
			print("Could not find any info on human with id " + str(num) )
		else :
			pass

def openAnimal(num):

	print("Press x to return to main orchard menu")

	for i in range(1,2):
		if(i == num):
			filePath = '../info/animal/animal' + str(num) +'.txt'
			
			#put wait to exit code here
			waitToExit(filePath)
			

		elif (i == 2 and i != num):
			print("Could not find any info on animal with id " + str(num) )
		else :
			pass

def openTractor(num):

	print("Press x to return main orchard menu")

	for i in range(1,2):
		if(i == num):
			filePath = '../info/tractor/tractor' + str(num) +'.txt'
			
			#put wait to exit code here
			waitToExit(filePath)
			

		elif (i == 2 and i != num):
			print("Could not find any info on tractor with id " + str(num) )
		else :
			pass











''' --------------------------- Excute code ----------------------------- ''' 
clear()
while True:
	c = Menu()

	if c == 'q':
		break
	else:
		loadFunction(c)

	#need to clear screen
	clear()

print("---------------------------------------------------------------")	
print("			     Finished")
print("---------------------------------------------------------------")
print("\n")
