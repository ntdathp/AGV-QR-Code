import serial
import utils 
import opencv
import time 

ser = serial.Serial(
	port = '/dev/ttyAMA0',
	baudrate = 9600,
	parity = serial.PARITY_NONE,
	stopbits = serial.STOPBITS_ONE,
	bytesize = serial.EIGHTBITS,
	timeout = 1
)
BACK = [0,0]
HEAD = [1,1]
LEFT = [0, 1]
RIGHT = [1, 0]

pNONE = 0
pRUN = 1
pCALIB1 = 2
pCALIB2 = 3
pCALIB3 = 4
pROTATELEFT = 5
pROTATERIGHT = 6
pBACK = 7
pDONE = 8

pCALIB4 = 9

ReadyFlag= 0


HeadFlag = 50
BackFlag = 51
Flag = None
list_process = [pRUN, pRUN, pROTATELEFT, pRUN]


print('out of uart')
class Uart:
    def ReceiveData(self): 
       
        global pRUN, pROTATELEFT, pCALIB1, pCALIB2, pCALIB3, pCALIB4, pDONE, pNONE, list_process
        global ReadyFlag
        
        print("into the receive_data")
        while 1:
            s = ser.readline() # s is an bytes object 
            data = s.decode('utf-8') # bytes object b'\xe2\x82\xac100' -> after using decode() method -> data: €100
            data = data.rstrip() # remove trailing whitespace ('\n')
            print(data)
            if "OK" in data:
                print('into OK condition')
                print(opencv.tProcess)
                if opencv.j == 1:
                    opencv.j = 0 
                    print('into j == 1 condition')
                    print(f"dvalue of i: {opencv.i}")
                    print(f"dvalue of list_process[i]: {list_process[opencv.i]}")
                    opencv.tProcess = list_process[opencv.i]
                    opencv.i += 1
                    if opencv.i == 4:
                        opencv.i = 0
                        opencv.tProcess = pDONE 
                if opencv.tProcess == pNONE:
                    pass
                elif opencv.tProcess == pCALIB1:
                    print('into opencv.tProcess == pCALIB1 condition')
                    utils.doAngle()
                elif opencv.tProcess == pCALIB2:
                    print('into opencv.tProcess == pCALIB2 condition')
                    utils.doDistance()
                elif opencv.tProcess == pCALIB3:
                    print('into opencv.tProcess == pCALIB3 condition') 
                    utils.doCorrect()
                elif opencv.tProcess == pCALIB4:
                    print('into opencv.tProcess == pCALIB4 condition')
                    utils.doDistanceCorrect()
                    
                elif opencv.tProcess == pRUN:
                    if opencv.j == 0: 
                        utils.doHead()
                    elif opencv.j == 3:
                        opencv.tProcess = pNONE
                        opencv.j = 0
                elif opencv.tProcess == pROTATELEFT:
                    if opencv.j == 0: 
                        utils.doRotateLeft()
                    elif opencv.j == 3:
                        opencv.tProcess = pNONE
                        opencv.j = 0