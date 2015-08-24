#author: Rommel Layco

#this code creates new blank files
#with the text no current info
#this is to ensure each time ros is run using rerun
#the debugger reads from a new file

def createPickerFiles():
	filepath = '../info/Pickers/picker'

	for i in range(1,8):
		path = filepath + str(i) + '.txt'
		f = open(path, 'w')
		f.write("No current information")
		f.close()



def createCarrierFiles():
	filepath = '../info/Carriers/carrier'

	for i in range(1,4):
		path = filepath + str(i) + '.txt'
		f = open(path, 'w')
		f.write("No current information")
		f.close()

def createHumanFiles():
	filepath = '../info/Humans/human'

	for i in range(1,3):
		path = filepath + str(i) + '.txt'
		f = open(path, 'w')
		f.write("No current information")
		f.close()

def createAnimalFiles():
	filepath = '../info/Animals/animal'

	for i in range(1,2):
		path = filepath + str(i) + '.txt'
		f = open(path, 'w')
		f.write("No current information")
		f.close()


''' ------------------------- Generate Files ------------------------------------------'''
createPickerFiles()
createCarrierFiles()
createHumanFiles()
createAnimalFiles()