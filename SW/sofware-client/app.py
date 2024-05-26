import opencv 
import uart 

import serial
import time  
import threading 
from threading import Thread 

thread_opencv = opencv.OpenCV()
doOpenCV = thread_opencv.doOpenCV()

thread_receivedata = uart.Uart()
ReceiveData = thread_receivedata.ReceiveData() 



try:
    t = time.time()
    t1 = threading.Thread(target=ReceiveData)
    t2 = threading.Thread(target=doOpenCV)
    t1.start()
    t2.start()
    t1.join()
    t2.join()
    print("All thread join!")
except KeyboardInterrupt:
    uart.ser.close()