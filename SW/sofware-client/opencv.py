import cv2 as cv 
import numpy as np 
from pyzbar.pyzbar import decode 
import time 
import AiPhile 
from math import atan2, pi
import utils 
import uart 

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

ReadyFlag = 0
angle_point = 0 # goc giua 2 diem T-G vs duong thang ve phia truoc
distance_correct = 0
distance = 0 
angle = 0

HeadFlag = 50
BackFlag = 51
Flag = None
i = 0 # bien check hanh dong tiep theo giua 2 QR code lien tiep
j = 0
list_process = [pRUN, pRUN, pROTATELEFT, pRUN]

tProcess = pNONE
print('out of opencv')
class OpenCV: 
    def doOpenCV(self):
        T_x = 315 
        T_y = 245
        global Flag, HeadFlag, BackFlag
        global angle_point, distance, angle, distance_correct
        global list_process
        global i
        global tProcess
        global pRUN, pROTATELEFT, pCALIB1, pCALIB2, pCALIB3
        cap = cv.VideoCapture(-1)
        _, frame = cap.read()
        old_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        lk_params = dict(winSize=(20, 20),
                        maxLevel=4,
                        criteria=(cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.01))
        cv.circle(frame, (265, 177), radius = 0, color = (255, 0, 0), thickness = 4)
        old_points = np.array([[]])
        qr_detected= False
        # stop_code=False
        frame_counter = 0
        starting_time = time.time()
        
        counter = 0 
        pre_angle = 500
        count_send = 0
        print("hehe")
        # SendData(1111,1111,1111,1,1)
        
        while 1:
            counter += 1
            frame_counter += 1
            ret, frame = cap.read()
            cv.line(frame, (65, 70), (565, 70), (255, 255, 0), 2)
            cv.line(frame, (565, 70), (565, 420), (255, 255, 0), 2)
            cv.line(frame, (565, 420), (65, 420), (255, 255, 0), 2)
            cv.line(frame, (65, 420), (65, 70), (255, 255, 0), 2)
            img = frame.copy()
            barcodes = decode(frame) 

            # cv.line(frame, (213, 0), (213, 479), (0, 255, 255), 1)

            for barcode in barcodes: 
                (x, y, w, h) = barcode.rect

                barcodeData = barcode.data.decode("utf-8") 
                barcodeType = barcode.type
                
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
                #print(points)
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
                    angle = angle * 180 / pi
                    

                    if pre_angle > angle - 2 or pre_angle < angle + 2: 
                        count_send += 1 
                       
                    pre_angle = angle
                   
                    G_x = (p2_x + p4_x) / 2.0
                    G_y = (p2_y + p4_y) / 2.0
                    

                    if tProcess == pCALIB4:
                        distance_correct = round(utils.ComputeDistance(p2_x, p2_y, p4_x, p4_y, T_x, T_y))
                        if G_y > 245.0:
                            Flag = BackFlag
                        else:
                            Flag = HeadFlag
                    
                    print(count_send)             
                    if count_send == 20:
                        count_send = 0
                        print('count_send reach 20')
                        
                        if tProcess == pNONE:
                            print('into tprocess == pNONE')
                            tProcess = pCALIB1
                            print(f'tProcess Value: {tProcess}')
                            angle_point = round(utils.ComputeAngle(p2_x, p2_y, p4_x, p4_y, T_x, T_y))
                            distance = round(utils.ComputeDistance(p2_x, p2_y, p4_x, p4_y, T_x, T_y))
                            utils.SendData(1111,1111,1111,1,1)
                            
            img = cv.resize(img, None, fx=2, fy=2,interpolation=cv.INTER_CUBIC)

            gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            stop_code = False
            if qr_detected and stop_code == False:
                # print('detecting')
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

            # if the 'c' key is pressed, break from the loop
            if key == ord("q"):
                break
            fps = frame_counter/(time.time()-starting_time)
            AiPhile.textBGoutline(frame, f'FPS: {round(fps,1)}', (30,40), scaling=0.6)
             #cv.imshow("Streaming", frame) # chỉ cần comment dòng này thì sẽ không show UI 

                # close all open windows
        #cv.destroyAllWindows()
        #cap.release()