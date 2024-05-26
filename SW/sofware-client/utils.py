from math import sqrt, atan2
from math import pi 
import uart 
import opencv 
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


def ComputeDistance(p2_x, p2_y, p4_x, p4_y, T_x, T_y):
    G_x = (p2_x + p4_x) / 2.0
    G_y = (p2_y + p4_y) / 2.0
    d = sqrt((T_x - G_x) ** 2 + (T_y - G_y) ** 2)
    return d 

def ComputeAngle(p2_x, p2_y, p4_x, p4_y, T_x, T_y):
    G_x = (p2_x + p4_x) / 2.0
    G_y = (p2_y + p4_y) / 2.0
    
    y = T_x - G_x 
    x = T_y - G_y 
    angle = atan2(y, x) # radian 
    # convert radian to degree 
    angle = angle * 180 / pi
    return angle 



def FixLength(value):
    if len(str(value)) == 1:
        return "000" + str(value)
    elif len(str(value)) == 2:
        return "00" + str(value)
    elif len(str(value)) == 3:
        return "0" + str(value)
    else: 
        return str(value)    


def FormatData(Pos, Vel, Acc, Dir1, Dir2):
    PosStr = FixLength(Pos)
    VelStr = FixLength(Vel)
    AccStr = FixLength(Acc)
    Dir1Str = str(Dir1)
    Dir2Str = str(Dir2)
    result = "{},{},{},{},{}".format(PosStr, VelStr, AccStr, Dir1Str, Dir2Str)
    return result


def SendData(Pos, Vel, Acc, Dir1, Dir2):
    data2Send = FormatData(Pos, Vel, Acc, Dir1, Dir2)
    print(data2Send)
    print("into the send_data")
    uart.ser.write(bytes(data2Send, 'utf-8'))
    uart.Uart().ReceiveData()
def doAngle():
    global pRUN, pROTATELEFT, pCALIB1, pCALIB2, pCALIB3, list_process
    #self.distance = opencv.distance
    print(round(opencv.angle_point * 4))
    print(f"distance {opencv.distance} in doangle")
    # Xoay sang trai 
    if opencv.angle_point > 0:
        angle_point = round(opencv.angle_point * 4)
        opencv.tProcess = pCALIB2
        SendData(angle_point, 20, 15, LEFT[0], LEFT[1])

    # Xoay sang phai
    else:
        angle_point = opencv.angle_point * 4
        angle_point = round(abs(angle_point))
        opencv.tProcess = pCALIB2
        SendData(angle_point, 20, 15, RIGHT[0], RIGHT[1])


def doDistance():

    global pRUN, pROTATELEFT, pCALIB1, pCALIB2, pCALIB3, list_process
    # Quy do d thanh <do xung>
    # d = d * hehe (thuc nghiem de tim duoc gia tri)

    distance = round(opencv.distance * 1.0 / 5 )
    print(f"{distance} in doDistance")
    # SendData(d, 30, 15, HEAD[0], HEAD[1])
    opencv.tProcess = pCALIB3
    SendData(distance, 20, 15, 1, 1)
    
def doDistanceCorrect():
    print(f"Flag is {opencv.Flag}")
    print('into doDistanceCorrect function')
    global pRUN, pROTATELEFT, pCALIB1, pCALIB2, pCALIB3, list_process
    distance_correct = round(opencv.distance_correct * 3.0 / 5 )
    print(f"distance_correct: {distance_correct}")
    if opencv.Flag == opencv.BackFlag:
        SendData(distance_correct, 20, 15, 0, 0)
    elif opencv.Flag == opencv.HeadFlag: 
        SendData(distance_correct, 20, 15, 1, 1)
    opencv.j = 1
    opencv.Flag = None
    


def doCorrect():
    global pRUN, pROTATELEFT, pCALIB1, pCALIB2, pCALIB3, list_process
    '''
    angle > 0: xoay sang trai (0->180)
    angle < 0: xoay sang phai (0->-180)
    '''
    print(f"{opencv.angle} in doCorrect")
     
    if opencv.angle > 0:  
        angle = round(opencv.angle * 4)
        print(f"do angle {angle}")
        SendData(angle-20, 20, 15, 0, 1)
    else: 
        angle = round(opencv.angle * 4)
        angle = abs(angle-20)
        print(f"do angle {angle}")
        #opencv.tProcess = list_process[i]
        SendData(abs(angle), 20, 15, 1, 0)
    opencv.tProcess = pCALIB4
    
    #time.sleep(2)# list_process = [pHead, pRotateLeft]
    # i += 1 
    
def doHead():
    SendData(495, 40, 10, HEAD[0], HEAD[1])
    opencv.j = 3
    

def doRotateLeft():
    SendData(360, 20, 15, LEFT[0], LEFT[1])
    opencv.j = 3