import cv2 as cv 
import numpy as np
from pyzbar.pyzbar import decode
import time
import AiPhile

import json 

import queue 
import utils
import serial 
import threading 
from math import atan2, pi, ceil

import random
from paho.mqtt import client as mqtt_client

############# variable - uart ###############
### global variable
BACK            = [0, 0]
HEAD            = [1, 1]
LEFT            = [0, 1]
RIGHT           = [1, 0]

pNONE           = 0
pCALIB1         = 1
pCALIB2         = 2
pCALIB3         = 3
pCALIB4         = 4
pCALIBDONE      = 5

pRUN            = 6
pROTATELEFT     = 7
pROTATERIGHT    = 8

pBACK           = 9
pDONE           = 10

pSTOP           = 11 
pBACK           = 12 
BackFlag        = 50
HeadFlag        = 51

p0              = 52 
p90             = 53
n90             = 54
n180            = 55


Head_doing      = 100 
Calib3_doing    = 101

tProcess        = pNONE
i               = 0


q = queue.Queue() # Init empty queue 
q2 = queue.Queue() # Data subcribe from GUI 
q3 = queue.Queue()
q4 = queue.Queue() # Data publish to GUI 
q5 = queue.Queue() # Queue cho phần Backward 
########### function - uart  ###################
ser = serial.Serial(
	port        = '/dev/ttyS0',
	baudrate    = 115200,
	parity      = serial.PARITY_NONE,
	stopbits    = serial.STOPBITS_ONE,
	bytesize    = serial.EIGHTBITS,
	timeout     = 1
)

########## The accessing information of the broker ######
broker = 'broker.emqx.io'
port = 1883
topic_sub = "AGV/AGV_01/control"
topic_pub = "AGV/AGV_01/feedback"
client_id = f'python-mqtt-{random.randint(0, 100)}'
username = 'thanh'
password = 'public'

def preprocessing(s):
    s = s[:17]
    split_string = s.split("|")
    list_process_string = split_string[0].split(",")
    calib_flag_string = split_string[1].split(",")
    '''
    Convert string to list 
    '''
    list_process = [int(i) for i in list_process_string]
    calib_flag = [int(i) for i in calib_flag_string]
    '''
    Remove 4 (None value) from list_process 
    '''
    while 4 in list_process: 
        list_process.remove(4)
    while 0 in list_process: 
        list_process.remove(0) 
    return list_process, calib_flag

def connect_mqtt() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker - Subcribe Mission!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def connect_mqtt_pub():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker - Publish Mission!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client 

def subscribe(client: mqtt_client):
    global q2 
    def on_message(client, userdata, msg):
        message = msg.payload.decode()
        message = preprocessing(message)
        # message = message.split()
        # message = [int(i) for i in message]
        if q2.qsize() == 0:
            q2.put(message)
        if q2.qsize() == 1: 
            q2.put("run")
    client.subscribe(topic_sub)
    client.on_message = on_message

def publish(client):
    # while True:
    if q4.qsize() == 1: 
        data = q4.get()
        time.sleep(1)
        message = data
        result = client.publish(topic_pub, message)
        status = result[0]
        if status == 0:
            print(message)
        else:
            print(f"Failed to send message to topic {topic_pub}")

def PubMessage():
    global q4 
    client = connect_mqtt_pub()
    client.loop_start()
    publish(client)

def SubMessage():
    global q2 
    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()


'''
Calib 1: điều chỉnh góc giữa tâm camera tâm mã QR cùng nằm trên đường Bắc của xe 
'''
def doAngle(q):
    global tProcess
    if q.qsize() == 3: 
        angle_point = q.get()
    angle_point = angle_point * 3.7
    if angle_point > 0:
        # goc (duong) be hon 90 -> xoay trai 
        if angle_point <= 90 * 3.7:
            angle_point = round(angle_point)
            tProcess = pCALIB2
            SendData(round(angle_point), 30, 20, LEFT[0], LEFT[1])
        # goc (duong) lon hon 90 -> xoay trai
        else: 
            angle_point = 180 * 3.7 - round(angle_point)
            tProcess    = pCALIB2 
            SendData(round(angle_point), 30, 20, RIGHT[0], RIGHT[1])
    # angle_point < 0 
    else: 
        angle_point = round(abs(angle_point))
        if angle_point <= 90 * 3.7: 
            tProcess = pCALIB2
            SendData(round(angle_point), 30, 20, RIGHT[0], RIGHT[1])
        else: 
            tProcess = pCALIB2 
            angle_point = 180 *3.7 - angle_point 
            SendData(round(angle_point), 30, 20, LEFT[0], LEFT[1])
'''
Calib 2: điều chỉnh khoảng cách giữa tâm camera và mã QR 
'''   
def doDistance(q):
    global tProcess
    if q.qsize() == 2:
        Flag = q.get()
        distance = q.get()
    # 1000 -> go back 
    if Flag == BackFlag:
        distance = round(distance * 4.0 / 5)
        tProcess = pCALIB3
        SendData(distance, 15, 10, 0, 0)
    # 1001 -> go head 
    elif Flag == HeadFlag:
        distance = round(distance * 2.0 / 5)
        tProcess = pCALIB3
        SendData(distance, 15, 10, 1, 1)
# param: angle    
def doCorrect(q): 
    print(f'into doCorrect function and size of q = {q.qsize()}')
    global tProcess
    if q.qsize() == 2:
        angle = q.get()
    if q.qsize() == 1:
        calib_flag = q.get()
    print(f"into doCorrect function, angle = {angle}, calib_flag = {calib_flag}")
    if calib_flag == 0:
        angle = round(angle * 3.7)
        if angle >= -1 and angle <= 1: 
            tProcess = pCALIB4 
            if q3.qsize() == 0:
                q3.put("terminate calib 3")
        elif angle > 1:
            tProcess = pCALIB4
            SendData(abs(angle-20), 20, 15, 0, 1)
        elif angle < -1: 
            angle = abs(angle)
            tProcess = pCALIB4
            SendData(abs(angle-20), 20, 15, 1, 0)

    if calib_flag == 2:
        if angle == -90: 
            tProcess = pCALIB4 
            if q3.qsize() == 0:
                q3.put("terminate calib 3")
        elif angle > -90:
            angle = round(angle * 3.7) 
            tProcess = pCALIB4 
            SendData(abs(angle + 335), 20, 15, 0, 1)
        elif angle < -90: 
            angle = round(angle * 3.7) 
            tProcess = pCALIB4 
            SendData(abs(angle + 335), 20, 15, 1, 0)

    if calib_flag == 1:
        if angle >= 89 and angle <= 91:
            tProcess = pCALIB4 
            if q3.qsize() == 0:
                q3.put("terminate calib 3")
        if angle > 91: 
            angle = round(angle * 3.7)
            tProcess = pCALIB4
            SendData(abs(angle - 335), 20, 15, 0, 1)
        elif angle < 89: 
            angle = round(angle * 3.7)
            tProcess = pCALIB4
            SendData(abs(angle - 335), 20, 15, 1, 0) 
    # if i < 4:
    if calib_flag == 3:
        if abs(angle) >= 179 and abs(angle) <= 180: 
            tProcess = pCALIB4
            if q3.qsize() == 0:
                q3.put("terminate calib 3") 
        elif angle < 0 and angle > -179:
            angle = round(angle * 3.7)
            tProcess = pCALIB4 
            SendData(abs(round(angle + 180 * 3.7)), 20, 15, 0, 1)
        elif angle > 0 and angle < 179: 
            angle = round(angle * 3.7)
            tProcess = pCALIB4
            SendData(abs(round(angle - 180 * 3.7)), 20, 15  , 1, 0)      
            # SendData(335, 40, 30, 1, 0)
def doCorrectDistance(q):
    global tProcess 
    print(f'into doCorrectDistance function and the size of q is {q.qsize()}') 
    if q.qsize() == 2: 
        Flag = q.get()
    print(f"Flag is {Flag}")
    if q.qsize() == 1: 
        distance = q.get()
    
    if Flag == 50:
        distance = round(distance * 5.0 / 5)
        tProcess = pCALIBDONE
        # tProcess = pSTOP 
        SendData(distance, 15, 10, 0, 0)
    
    else:
        distance = round(distance * 1.0 / 5)
        tProcess = pCALIBDONE
        # tProcess = pSTOP  
        SendData(distance, 15, 10, 1, 1)

def doHead(q):
    global tProcess 
    SendData(495, 50, 40, 1, 1)
    # Nếu gán tProcess = pNONE ngay ở đây thì ngay lập tức luồng doOpenCV sẽ thực hiện CALIB 
    tProcess = pDONE 
    if q.qsize() == 0: 
        q.put(Head_doing)

def doBack(q):
    global tProcess 
    SendData(670, 30, 20, 0, 1)
    if q5.qsize() == 0: 
        q5.put("go cablib 1")
def doRotateLeft(q):
    global tProcess 
    tProcess = pCALIBDONE
    SendData(335, 40, 30, 0, 1)

def doRotateRight(q): 
    global tProcess 
    tProcess = pCALIBDONE 
    SendData(335, 40, 30, 1, 0)

def SendData(Pos, Vel, Acc, Dir1, Dir2):
    data2Send = utils.FormatData(Pos, Vel, Acc, Dir1, Dir2)
    print(data2Send)
    print("into the SendData function")
    ser.write(bytes(data2Send, 'utf-8'))

def ReceiveData(q):
    print('into RecieveData thread')
    global tProcess 
    while True:
        s = ser.readline() # s is an bytes object 
        data = s.decode('utf-8') # bytes object b'\xe2\x82\xac100' -> after using decode() method -> data: €100
        data = data.rstrip() # remove trailing whitespace ('\n')
        print(data) 
        if "OK" in data:
            print('into OK condition')
            print(f"tProcess = {tProcess}")
            if q5.qsize() == 1: 
                print(q5.get())
                tProcess = pNONE 

            if tProcess == pNONE:
                pass
            elif tProcess == pSTOP: 
                pass 
            elif tProcess == pDONE: 
                if q.get() == Head_doing: 
                    tProcess = pCALIB1 
                           
            elif tProcess == pCALIB1: # pCALIB1 = 2 
                # print(f'into tProcess == pCALIB1 condition and size of q = {q.qsize()}')
                if q.qsize() == 3: 
                    doAngle(q)
                else: 
                    continue 
            elif tProcess == pCALIB2: # pCALIB2 = 3 
                # print(f'into tProcess == pCALIB2 condition and size of q = {q.qsize()}')
                if q.qsize() == 2:
                    doDistance(q)
                else: 
                    continue 
            elif tProcess == pCALIB3: # pCALIB3 = 4 
                # print(f'into tProcess == pCALIB3 condition and size of q = {q.qsize()}') 
                if q.qsize() == 2:
                    doCorrect(q)
                else:  
                    # print(f"RecieveData into tProcess == pCALIB3 but can't do function doCorrect(q) because q.qsize()={q.qsize()}")
                    continue  
            elif tProcess == pCALIB4: # pCALIB4 = 9 
                # print(f'into tProcess == pCALIB4 condition and size of q = {q.qsize()}')
                if q.qsize() == 2:
                    doCorrectDistance(q)
                else: 
                    continue
            elif tProcess == pRUN:
                # print(f'into tProcess == pRUN condition and size of q = {q.qsize()}')
                if q.qsize() == 0:
                    doHead(q)
                else: 
                    continue 
            elif tProcess == pROTATELEFT: 
                # print(f"into tProcess == pROTATELEFT condition and size of q = {q.qsize()}")
                if q.qsize() == 0:
                    doRotateLeft(q)
                else: 
                    continue 
            elif tProcess == pROTATERIGHT:
                if q.qsize() == 0:
                    doRotateRight(q)
                else:
                    continue 
            elif tProcess == pBACK: 
                if q.qsize() == 0:
                    doBack(q)
                else:
                    continue 
            

def doOpenCV(q, q2):
    
    global tProcess
    global i                  
    print('into doOpenCV thread')
    angle_point         = 0 
    distance            = 0     
    angle               = 0
    T_x                 = 315 
    T_y                 = 245
    counter             = 0 
    pre_angle           = 500
    count_send          = 0
    count               = 0
    calib_mode          = 0 
    run                 = None 
    pre_message         = None 
    list_process        = None 
    list_process_backward = None 

    # list_process        = [6, 6, 7, 6]
    # calib_process       = [0, 2]
    # list_process_iter = iter(list_process)
    cap = cv.VideoCapture(-1)
    _, frame = cap.read()
    old_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    lk_params = dict(winSize=(20, 20),
                    maxLevel=4,
                    criteria=(cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.01))
    cv.circle(frame, (265, 177), radius = 0, color = (255, 0, 0), thickness = 4)
    old_points = np.array([[]])
    qr_detected= False
    frame_counter = 0
    starting_time = time.time()
    
    while True:
        if q2.qsize() == 2:
            list_process, calib_process = q2.get() # [pRUN, pRUN, pROTATELEFT, pRUN]
            if 7 in list_process: 
                list_process_backward = [8 if i == 7 else i for i in list_process]
                list_process_backward.reverse()
            elif 8 in list_process: 
                list_process_backward = [7 if i == 8 else i for i in list_process]
                list_process_backward.reverse()
            list_process.append(12)
            list_process_iter   = iter(list_process)
            
            list_process_backward_iter = iter(list_process_backward)
            calib_process_backward = [calib_process[1], calib_process[0]]
            
            if calib_process_backward[0] == 0:
                calib_process_backward[0] = 3
            elif calib_process_backward[0] == 3:
                calib_process_backward[0] = 0
            elif calib_process_backward[0] == 1:
                calib_process_backward[0] = 2
            elif calib_process_backward[0] == 2:
                calib_process_backward[0] = 1
        
            if calib_process_backward[1] == 0:
                calib_process_backward[1] = 3
            elif calib_process_backward[1] == 3:
                calib_process_backward[1] = 0
            elif calib_process_backward[1] == 1:
                calib_process_backward[1] = 2
            elif calib_process_backward[1] == 2:
                calib_process_backward[1] = 1

            print(f"list_process = {list_process}")
            print(f"calib_process = {calib_process}")
        if q2.qsize() == 1:
            run = q2.get()
        
        if list_process: 
            count_pub = 1
            counter += 1
            frame_counter += 1
            _, frame = cap.read()
            # cv.line(frame, (65, 70), (565, 70), (255, 255, 0), 2)
            # cv.line(frame, (565, 70), (565, 420), (255, 255, 0), 2)
            # cv.line(frame, (565, 420), (65, 420), (255, 255, 0), 2)
            # cv.line(frame, (65, 420), (65, 70), (255, 255, 0), 2)
            img = frame.copy()
            barcodes = decode(frame) 
            for barcode in barcodes: 
                barcodeData = barcode.data.decode("utf-8")
                try: 
                    barcodeData = json.loads(barcodeData)
                    data = str(barcodeData["Row"]) + "," + str(barcodeData["Column"])
                except:
                    pass 
                # forward 
                if count == 0:  
                    message = "DATA|0," + data
                    if pre_message != message:
                        if q4.qsize() == 0:
                            q4.put(message)
                        if q4.qsize() == 1: 
                            client = connect_mqtt_pub()
                            publish(client)     
                        pre_message = message 
                # backward 
                if count == 1:
                    message = "DATA|1," + data 
                    if pre_message != message: 
                        if q4.qsize() == 0:
                            q4.put(message)
                        if q4.qsize() == 1: 
                            client = connect_mqtt_pub()
                            publish(client)     
                        pre_message = message 
                # print(data)
                if len(barcode.polygon) != 4: 
                    pass 
                else:
                    p1_x = barcode.polygon[0].x
                    p1_y = barcode.polygon[0].y
                    
                    p2_x = barcode.polygon[1].x 
                    p2_y = barcode.polygon[1].y

                    p3_x = barcode.polygon[2].x 
                    p3_y = barcode.polygon[2].y

                    p4_x = barcode.polygon[3].x 
                    p4_y = barcode.polygon[3].y
                
                (rv, points, straight_qrcode) = cv.QRCodeDetector().detectAndDecode(frame)
                if rv:
                    points = points[0]

                    # Toa do dung cua ma QR
                    pt1 = points[0] 
                    pt2 = points[1]
                    pt3 = points[2]
                    pt4 = points[3]
                    a = int(pt2[1])
                    b = int(pt1[1])
                    c = int(pt2[0])
                    d = int(pt1[0])
                    angle = atan2(b - a, c - d)
                    angle = (angle * 180 / pi) 


                    
                    if pre_angle > angle - 2 or pre_angle < angle + 2: 
                        
                        count_send += 1 
                    pre_angle = angle
                    # print(f"**************G_x - 315   = {G_x - 315}")
                    # print(f"count_send = {count_send}")
                    # print(f"G_y = {(p2_y + p4_y) / 2.0}")
                    # print(f"angle_point = {round(utils.ComputeAngle(p2_x, p2_y, p4_x, p4_y, T_x, T_y))}")
                    if count_send == 5:
                        # print(f"angle is {angle}")
                        # print(f"count is {count}")
                        count_send = 0

                        if tProcess == pNONE:
                            print('doOpenCV into tprocess == pNONE')
                            tProcess = pCALIB1
                            # tProcess = pSTOP 
                        elif tProcess == pCALIB1:
                    
                            angle_point = round(utils.ComputeAngle(p2_x, p2_y, p4_x, p4_y, T_x, T_y))
                            distance = round(utils.ComputeDistance(p2_x, p2_y, p4_x, p4_y, T_x, T_y))
                            G_x = (p2_x + p4_x) / 2.0
                            # print(f'doOpenCV into tProcess == pCALIB1 and distance = {distance}')
                            '''
                            Goc 20 y nghia la gi ?
                            + Neu angle_point nho hon 20 thi bo qua hai buoc pCALIB1 (angle_point) va pCALIB2 (distance)
                            + Con lon hon 20 thi cho di 
                            '''

                            print(f"**************angle_point = {angle_point}")
                            print(f"**************distance    = {distance}")
                            print(f"**************angle       = {angle}")
                            print(f"**************G_x - 315   = {G_x - 315}")
                            if abs(G_x - 315) > 80:
                                if q.qsize() == 0:
                                    q.put(int(angle_point))   

                                if q.qsize() == 1:
                                    if abs(angle_point) >= 90:
                                        q.put(BackFlag) # di lui, goc lon hon 90
                                    elif abs(angle_point) < 90: 
                                        q.put(HeadFlag) # di toi, goc nho hon 90 
                                if q.qsize() == 2: 
                                    q.put(int(distance)) 
                        
                                SendData(1111,1111,1111,1,1)       
                            else: 
                                tProcess = pCALIB3
                                if q3.qsize() == 0: 
                                    q3.put("terminate")
                        #
                        # Calib flag nên quăng vào trong này để xử lý 
                        # Ý tưởng để xử lý: 
                        #                    
                        elif tProcess == pCALIB3:
                            # print("doOpenCV into tProcess == pCALIB3")
                            # print(f"doOpenCV tProcess = {tProcess} and angle = {angle} and size of q = {q.qsize()}")
                            if q.qsize() == 0: 
                                q.put(int(angle))
                            if q.qsize() == 1: 
                                if count == 0: 
                                    if calib_mode == 0: 
                                        q.put(calib_process[0])
                                    if calib_mode == 1: 
                                        q.put(calib_process[1])
                                elif count == 1: 
                                    if calib_mode == 0: 
                                        q.put(calib_process_backward[0])
                                    if calib_mode == 1: 
                                        q.put(calib_process_backward[1])
                            if q.qsize() == 2 and q3.qsize() == 1:
                                print(q3.get())
                                print(f"size of q after geting data {q3.qsize()}")
                                SendData(1111,1111,1111,1,1)  
                        
                        elif tProcess == pCALIB4:
                            count_calib4 = 0 
                            distance = round(utils.ComputeDistance(p2_x, p2_y, p4_x, p4_y, T_x, T_y))
                            # print(f"into tProcess == pCALIB4 and q.qsize() = {q.qsize()}, q3.qsize() = {q3.qsize()}")
                            # print(f"G_y = {G_y}")
                            # print(f"abs(angle) = {abs(angle)}")
                            if (abs(angle) > 170) or (abs(angle) > 80 and abs(angle) < 100) or (abs(angle) < 10):
                                G_y = (p2_y + p4_y) / 2.0
                                print(f"G_y = {G_y}")
                                count_calib4 += 1     
                                if q.qsize() == 0: 
                                    if G_y > 230:
                                        q.put(50) # 50 is BackFlag
                                    else:
                                        q.put(51) # 51 is HeadFlag 
                                if q.qsize() == 1: 
                                    q.put(abs(G_y - 245))
                                if q.qsize() == 2 and q3.qsize() == 1:
                                    print(q3.get()) 
                                    SendData(1111,1111,1111,1,1)
                                elif q.qsize() == 2 and count_calib4 == 10:
                                    count_calib4 = 0 
                                    SendData(1111,1111,1111,1,1)

                        # Giữa hai hành động trong list_process là quá trình calib
                        elif tProcess == pCALIBDONE:
                            # Lý do count == 1 nằm phía trên để chứ không phải dưới count == 0 -> ngẫm một tí là hiểu 
                            if count == 1: 
                                tProcess = next(list_process_backward_iter)
                                if tProcess == 7 or tProcess == 8:
                                    calib_mode = 1 

                            if count == 0: 
                                tProcess = next(list_process_iter)                    
                                if tProcess == 7 or tProcess == 8: 
                                    calib_mode = 1 
                                if tProcess == 12: 
                                    count = 1 
                                    calib_mode = 0 
                                    pre_message = None 

                                

            img = cv.resize(img, None, fx=2, fy=2,interpolation=cv.INTER_CUBIC)
            gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            stop_code = False
            if qr_detected and stop_code == False:
                new_points, status, error = cv.calcOpticalFlowPyrLK(old_gray, gray_frame, old_points, None, **lk_params)
                old_points = new_points 
                new_points = new_points.astype(int)
                n = (len(new_points))
                frame = AiPhile.fillPolyTrans(frame, new_points, AiPhile.GREEN, 0.4)
                AiPhile.textBGoutline(frame, f'Detection: Optical Flow', (30,80), scaling=0.5,text_color=AiPhile.GREEN)
                cv.circle(frame, (new_points[0]), 3,AiPhile.GREEN, 2)
            old_gray = gray_frame.copy()
            # press 'r' to reset the window
            key = cv.waitKey(1)
            if key == ord("s"):
                cv.imwrite(f'reference_img/Ref_img{frame_counter}.png', img)
            if key == ord("q"):
                break
            fps = frame_counter/(time.time()-starting_time)
            AiPhile.textBGoutline(frame, f'FPS: {round(fps,1)}', (30,40), scaling=0.6)
            # cv.imshow("Streaming", frame)
        # cv.destroyAllWindows()
        # cap.release()
try:
    t = time.time()
    t1 = threading.Thread(target=ReceiveData, args=(q, ))
    t2 = threading.Thread(target=doOpenCV, args=(q, q2))
    t3 = threading.Thread(target=SubMessage)
    # t4 = threading.Thread(target=PubMessage)
    t1.start()
    t2.start()
    t3.start()
    # t4.start()
    t1.join()
    t2.join()
    t3.join()
    # t4.join()
    print("All thread join!")
except KeyboardInterrupt:
    ser.close()