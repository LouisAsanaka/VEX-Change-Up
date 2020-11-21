'''
Currently one edit ahead of latest release:
1. Replaced call to plot function in single run function
'''

from tkinter import *
from tkinter import filedialog
import time
from time import gmtime, strftime
import sys
import serial
import serial.tools.list_ports
import matplotlib
import matplotlib.pyplot as plt
import mpl_toolkits
import os
import webbrowser

def printToConsole(outputBox, output):
    '''
    Prints out to the tkinter window on line 1
    '''
    outputBox.delete(1.0, "end")
    outputBox.insert("end", output)

    
def preProcessData(incomingDatum):
    '''
    takes in a single line from the serial 
    stream (with all the mumbo jumbo) and clean it up. Decode
    using utf-8 encoding, strip of any newline characters that
    may be present, and save data between the start character({) and
    the stop character (}) to a list.
    '''
    incomingDatum = incomingDatum.decode('latin1')
    incomingDatum = incomingDatum.strip('\n')
    incomingDatum = incomingDatum[6:]
    print(incomingDatum)

    #In the case that we are receiving steady empty string, then no start and stop characters are found
    try:
        incomingDatum = incomingDatum[incomingDatum.index('{')+len('{'):incomingDatum.index('}')] 
    except:
        #Replace datum with blank string
        incomingDatum = ''
    
    #splits incoming values at the comma, packs into a list
    incomingDatum = incomingDatum.split(',')
    return incomingDatum

class Run:
    '''
    This class is made for each time the file is run. The class
    contains information and functions for setting up and running
    a data collection cycle
    '''
    def __init__(self):
        self.usesStopChar = False
        self.testLen = None
        self.comPort = None
        self.dataStream = None
        self.xAxis = []
        self.fullDataSet = []
        self.connectionFlag = False

        self.okForGo = False

        self.comPort = comPortInput.get()
        self.usesStopChar = not(usesDuration.get())
        if(self.usesStopChar == False):
            try:
                self.testLen = float(testLenEntryBox.get())
            except:
                _testLenEntryError = popUpError(200, 150, "Enter a number for test duration")
    
        try:
            self.dataStream = serial.Serial(self.comPort, baudrate = 115200,timeout = 1) #Set to maximum baud for pReCIsIoN.
            self.okForGo = True

        except:
            pass
            _dataStreamError = popUpError(200, 150, "bad data stream")

    def waitForStart(self):
        '''
        This function blocks code until the "START" command is received.
        '''
        if(self.okForGo):
            #This loop waits for the START Command from V5 Brain
            startTime = time.time()
            while (time.time() - startTime <= 30.0):
                printToConsole(textConsole, "Waiting for data stream to start. (" + str(int(time.time() - startTime)) + "/30)")
                if(preProcessData(self.dataStream.readline())[0] == 'START'): #I know (is) is preferred to == but for some reason only == works.
                    printToConsole(textConsole, "Start command received. Starting Collection")
                    mainWindow.update()
                    return True
                mainWindow.update()
            _timeOutError = popUpError(200, 150, "Time Out!")
            printToConsole(textConsole, " ")
            return False
        else:
            return False
    
    def loop(self):
        '''
        Main loop for the data collection sequence. May be exited by
        either duration or receiving the 'STOP' command, depending on
        variables in settings.
        '''
        #Failure variable for looping
        lineDropCount = 0

        #main loop
        #timed run counter
        startTime = time.time()
        while(self.usesStopChar or time.time() - startTime < self.testLen):

            #Runs if data stream has been opened
            if (self.dataStream.is_open):

                #unpacks and decodes serial lines
                incomingDatum = preProcessData(self.dataStream.readline())
                if(incomingDatum[0]=="STOP"):
                    printToConsole(textConsole, "Stop Command Received.")
                    time.sleep(1)
                    break
                printToConsole(textConsole, str(int(time.time() - startTime)))
                #Convert list of strings into a list of float
                try:
                    for stringDatumIndex in range(len(incomingDatum)):
                        incomingDatum[stringDatumIndex] = float(incomingDatum[stringDatumIndex])
                                
                    self.xAxis.append(incomingDatum[0]) #Apppend the x-series value to our x-list
                    incomingDatum.pop(0) #Delete x-series value from the datum list
                    self.fullDataSet.append(incomingDatum) #append the n-series' of single element list to full set
                except:
                    lineDropCount += 1
                    if (lineDropCount > 50):
                        printToConsole(textConsole, "Too many errors. Stopping collection")
                        sys.exit()

    def plot(self):
        '''
        Uses Matplotlib.pyplot to plot the data set after looping.
        '''
        plt.plot(self.xAxis, self.fullDataSet)
        if (savePlot.get()):
            plt.savefig(saveDirectory + "/" + strftime('%Y-%m-%d-%H-%M', gmtime()) + ".pdf")
            print(saveDirectory + "/" + strftime('%Y-%m-%d-%H-%M', gmtime()) + ".pdf")
            pass
        plt.show()

def performSingleRun():
    currentRun = Run()
    if (currentRun.waitForStart()):
        currentRun.loop()
        currentRun.plot()

    del currentRun
    #print("current Run Deleted")


class popUpError:
    def __init__(self, sizeX, sizeY, message):
        self.errorObject = Tk()
        self.errorObject.geometry(str(sizeX) + "x"+str(sizeY))
        self.errorObject.title("Error!")
        self.errorText = Label(self.errorObject, text = message)
        self.errorText.place(relx = .5, rely = .5, anchor = CENTER)


def createDurationEntry():
    '''
    Create the duration entry box if user selects the 
    use Duration checkbox
    '''
    global testLenEntryBox
    #Create the settings title
    durationText = Label(text = 'Enter the duration')
    durationText.place(anchor = 'w', rely = horiz1, relx=vert3)

    #Create the entry box for entering duration
    testLenEntryBox.place(anchor = 'w', rely = horiz1, relx = vert4)

global saveDirectory
saveDirectory = os.path.abspath(os.getcwd())
#print(saveDirectory)
def enterSavePath():
    '''
    Function to create instance of the save
    path class. Gets called from file menu.
    '''
    global saveDirectory
    saveDirectory = filedialog.askdirectory()
    #print(saveDirectory)

def openGitHub():
    '''
    Opens the Git repo when help is hit.
    '''
    url = 'https://github.com/adityanarayanan03/V5SerialPlotter/blob/master/readme.md'
    webbrowser.open(url, new=0, autoraise=True)

#Define some variables for positioning
#Percentages of frame size for relx and rely command
horiz1 = 0.035
horiz2 = 0.095
horiz3 = 0.12
vert1 = 0.025
vert2 = .30
vert3 = .6
vert4 = .75

#Create the full window that user will see
mainWindow = Tk()
mainWindow.geometry("700x700")
mainWindow.title("V5 Serial Plotter")

#This block is all for creating the File and Help menu at the top of the window
menuBar = Menu(mainWindow)
fileMenu = Menu(menuBar, tearoff = 0)
fileMenu.add_command(label = "Edit Save Path", command = enterSavePath)
fileMenu.add_separator()
helpMenu = Menu(menuBar, tearoff = 0)
helpMenu.add_command(label = "About", command = openGitHub)
menuBar.add_cascade(label="File", menu=fileMenu)
menuBar.add_cascade(label = "Help", menu = helpMenu)
mainWindow.config(menu = menuBar)


#Create the drop-down menu for Serial port selection
comPortInput = StringVar(mainWindow)
comPortInput.set("Select Serial Port") #Setting Default Value
comPortOptions = [comport.device for comport in serial.tools.list_ports.comports()]
if not comPortOptions:
    comPortOptions = ['NONE']
comPortSelector = OptionMenu(mainWindow, comPortInput, *comPortOptions)
comPortSelector.place(anchor='w', rely = horiz1, relx = vert1)

#Create the save plot selector
savePlot = BooleanVar()
savePlotCheckButton = Checkbutton(mainWindow, text = 'Save Plot', variable= savePlot)
savePlotCheckButton.place(anchor = 'e', rely = horiz3, relx = .45)

#Create the checkbox to use duration-based test
usesDuration = BooleanVar()
usesDurationCheckButton = Checkbutton(mainWindow, text = 'Time-based data collection', variable = usesDuration, command = createDurationEntry)
usesDurationCheckButton.place(anchor = 'w', rely = horiz1, relx = vert2)

testLenEntryBox = Entry(mainWindow)

#Create a run button
runButton = Button(text = 'Run', command = performSingleRun, width = 7, height = 1)
runButton.place(rely=horiz3, relx = .55, anchor = 'w')

#Create a text console for output
textConsole = Text(mainWindow, width = 75, height = 3)
textConsole.place(rely = .3, relx = .5, anchor = CENTER)

mainWindow.mainloop()