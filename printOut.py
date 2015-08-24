def Menu ():
	print("---------------------------------------------------------------")
	print("			Orchard Debugger		")
	print("---------------------------------------------------------------")
	print("please enter the node you want to check")


	#continue looping till a valid input is entered
	while True:
		try:
			char =  input("p = picker, c = carrier, h = human, a = animal: ")
		except ValueError:
			#loop again invalid type
			print("Input is not a char")
			continue
		else:
			if char == 'p' or char == 'c' or char == 'h' or char == 'a' :
				break
			else :
				print("You did not select a valid char")
				print("Try again\n")
				print("Select a char from the following")
				
	
	return char


def loadFunction(char):
	if char == 'p':
		num = choosePicker()
		line = openPicker(num)
	elif char == 'c':
		num  = chooseCarrier()
		#line = openCarrier()
		


''' -------------------------------------- choose robot number -----------------------'''

def choosePicker():
	print('\n')
	print("---------------------------------------------------------------")
	print("			Picker Menu		")
	print("---------------------------------------------------------------")
	print("please enter the picker number you want to check")


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
	print("---------------------------------------------------------------")
	print("please enter the carrier number you want to check")


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
				
				
	
	return num

''' --------------------------- Read File -------------------------------------------------------------'''
def openPicker(num):

	for i in range(1,8):
		if(i == num):
			filePath = 'info/picker' + str(num) +'.txt'
			f = open(filePath, 'r')
			lastline = f.readlines()[-1]
			print(lastline)
		elif (i == 7 and i != num):
			print("Could not find any info on picker " + str(num) )
		else :
			pass

def openCarrier(num):
	for i in range(1,11):
		if(i == num):
			filePath = 'info/carrier' + str(num) +'.txt'
			f = open(filePath, 'r')
			lastline = f.readlines()[-1]
			print(lastline)
		elif (i == 7 and i != num):
			print("Could not find any info on carrier " + str(num) )
		else :
			pass











''' --------------------------- Excute code ----------------------------- ''' 
c = Menu()

loadFunction(c)

	
print("Finished")