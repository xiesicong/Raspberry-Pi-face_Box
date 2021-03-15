# -*- coding: utf-8 -*-


#--------------------------------------------------------------------------------email相关---------------------#start
from __future__ import division

import time  
import signal  
import sys
reload(sys)
sys.setdefaultencoding('utf8')
#-----------------------------------以上为控制编码方式
import smtplib                                  #引入SMTP协议包
from email.mime.text import MIMEText
from email.header import Header
from email.mime.multipart import MIMEMultipart  #创建包含多个部分的邮件体
from email.mime.base import MIMEBase            #添加附件（附件内容并附加到根容器 ）
from email.mime.image import MIMEImage
import os.path                                  #分析路径
from email import Encoders

OldTimeStamp=0                                      
sender = "2861725742@qq.com"                    #发送邮箱，qq邮箱
password = "oeilvfvudfzbdfcc"
receiver = "1634876788@qq.com"                  #目标邮箱
#--------------------邮件服务与端口信息----------------------
smtp_server = "smtp.qq.com"
smtp_port = 465                                 #qq的SMTP端口465
msg = MIMEMultipart('related')                  #采用related定义内嵌资源的邮件体
#--------------------------------------------------------------------------------email相关---------------------#end


import cv2                                                                                                      #导入库
import numpy as np
from PIL import Image
import os 
import RPi.GPIO as GPIO

#--------------------------------------------------------------------------------串口相关---------------------#start
import serial
ser = serial.Serial("/dev/ttyAMA0", 115200)

uart_input = '0'                                                                                                #串口接收初始化
uart_input_buff = '0'                                                                                           #串口接收缓存初始化

KNN_uart_flag=1                                                                                                 #KNN中串口发送标识
REC_uart_flag=1                                                                                                 #REC中串口发送标识

KNN_Warning_old_time = 0                                                                                        #KNN中上一次警报的时间，防止警报太频繁
#--------------------------------------------------------------------------------串口相关---------------------#end


#--------------------------------------------------------------------------------GPIO相关---------------------#start
GPIO.setmode(GPIO.BOARD)                                                                                        #GPIO模式
GPIO.setup(29, GPIO.IN)                                                                                         #设为输入模式
#--------------------------------------------------------------------------------GPIO相关---------------------#end


#--------------------------------------------------------------------------------人脸相关---------------------#start
#获取人脸
face_detector = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')                                    #级联分类器，支持Haar特征
#训练
path = 'dataset'                                                                                                #需要训练的图片路径
recognizer = cv2.face.LBPHFaceRecognizer_create()                                                               #特征提取方式，能提取出图像的局部的纹理特征
detector = cv2.CascadeClassifier("haarcascade_frontalface_default.xml");                                        #级联分类器，支持Haar特征
#识别
recognizer = cv2.face.LBPHFaceRecognizer_create()                                                               #特征提取方式，能提取出图像的局部的纹理特征
recognizer.read('trainer/trainer.yml')                                                                          #读取用户人脸训练集
cascadePath = "haarcascade_frontalface_default.xml"                                                             #opencv自带的训练后的人脸检测数据,可用于进行人脸脸部检测
faceCascade = cv2.CascadeClassifier(cascadePath);                                                               ##级联分类器，支持Haar特征

font = cv2.FONT_HERSHEY_SIMPLEX                                                                                 #字体

id = 0                                                                                                          #iniciate id counter

names = ['None', 'xsc1', 'xsc2', 'xsc3', 'Z', 'W']                                                              #名字表

cam = cv2.VideoCapture(0)                                                                                       #初始化并开始实时视频捕获
cam.set(3, 320)                                                                                                 #设置窗口宽
cam.set(4, 240)                                                                                                 #设置窗口高

minW = 0.1*cam.get(3)                                                                                           #为人脸识别定义最小窗口大小
minH = 0.1*cam.get(4)
#--------------------------------------------------------------------------------人脸相关---------------------#end


#--------------------------------------------------------------------------------模式相关---------------------#start
flag_add = 0                                                                                                    #添加人脸标识，为1时运行添加人脸程序
flag_mode = 0                                                                                                    #识别标识，为0时运行KNN警觉代码，为1时运行人脸识别
bs=cv2.createBackgroundSubtractorKNN(detectShadows=True)                                                        # KNN
KNN_run_flag = 0                                                                                                #KNN运行标识
REC_run_flag = 0                                                                                                #REC运行标识
email_send_cnt = 0                                                                                              #陌生人计数，检测到一定次数的陌生人次数才发邮件
#--------------------------------------------------------------------------------模式相关---------------------#end



# function to get the images and label data                                                                     #获取图像和标签数据的函数
def getImagesAndLabels(path):

    imagePaths = [os.path.join(path,f) for f in os.listdir(path)]                                               #os.listdir() 方法用于返回指定的文件夹包含的文件或文件夹的名字的列表
                                                                                                                #os.path.join(path,f)把一个文件路径的字符串连接起来，形成这个f文件的完整的绝对路径
    faceSamples=[]
    ids = []

    for imagePath in imagePaths:

        PIL_img = Image.open(imagePath).convert('L') # convert it to grayscale                                  #将其转换为灰度
        img_numpy = np.array(PIL_img,'uint8')                                                                   #建立数组

        id = int(os.path.split(imagePath)[-1].split(".")[1])                                                    #split()：拆分字符串。通过指定分隔符对字符串进行切片，并返回分割后的字符串列表。
                                                                                                                #os.path.split()：将文件名和路径分割开。
                                                                                                                
        faces = detector.detectMultiScale(img_numpy)                                                            #它可以检测出图片中所有的人脸
                                                                                                                #并将人脸用vector保存各个人脸的坐标、大小（用矩形表示）

        for (x,y,w,h) in faces:
            faceSamples.append(img_numpy[y:y+h,x:x+w])                                                          #list.append(object)向列表中添加一个对象object
            ids.append(id)

    return faceSamples,ids

#串口接收发送
# def main():
    # while True:
        # recv = get_recv()                       #检测接收
        # if recv != None:                        #如果接收不为空
            # print recv                          #打印到树莓派窗口
            # ser.write(recv[0] + "\r\n")           #把发过来的内容发回去
        # time.sleep(0.1)
    
#串口接收   
def get_recv():
    cout = ser.inWaiting()                      #等待接收
    if cout != 0:                               #接受不为空
        line = ser.read(cout)                   #读取接受的内容
        recv = str.split(line)                  #分割字符串方法
        ser.reset_input_buffer()                #看名字，应该是清除输入缓存
        return recv                             #返回字符串

ser.write("RespberryPi OK" + "\r\n")

while True:
    # time.sleep(0.1)

    uart_input = get_recv()                                                                                     #检测接收
    if uart_input != None:                                                                                      #如果接收不为空
        print uart_input[0]                                                                                     #打印到树莓派窗口
        uart_input_buff = uart_input[0]                                                                         #进行缓存
        ser.write(uart_input[0] + "\r\n")                                                                       #把发过来的内容发回去


        
    if uart_input_buff == 'addface':                                                                            #是否接收到addface
        flag_add=1                                                                                              #添加人脸标识置1，开始添加人脸
        uart_input_buff = '0'                                                                                   #清除缓存
    # elif uart_input_buff == 'KNN':
        # flag_mode=0
        # flag_add=0                                                                                              #识别的时候添加人脸要关掉，否则优先进入添加人脸
        # uart_input_buff = '0'
    # elif uart_input_buff == 'REC':
        # flag_mode=1
        # flag_add=0
        # uart_input_buff = '0'
    
    if GPIO.input(29):                                                                                         #识别标识，为0时运行KNN警觉代码，为1时运行人脸识别
        flag_mode = 1
        flag_add=0
    else:
        flag_mode = 0
        flag_add=0
    

    if flag_add==1:                                                                                             #添加人脸
    
        cv2.destroyAllWindows()
        
        # face_id = input('\n enter user id end press <return> ==>  ')                                                  #为当前人脸数据集设置一个id
        # print(type(face_id))
        ser.write("enter user id end press <return> ==>  ")
        
        buff = get_recv()                                                                                               #获取串口接收数据
        while buff == None:                                                                                             #判断是否接收为空，为空的话继续接收
            buff = get_recv()
        face_id = int(buff[0])                                                                                          #这里[0]是为了取buff列表中的字符串，然后用int将字符串强转为整型
        ser.write( buff[0]+ "\r\n")
        
        print('\n  开始添加人脸，请对准摄像头......')
        print("\n [INFO] Initializing face capture. Look the camera and wait ...")                                      #打印输出，提示用户看摄像头
        ser.write("Getting faces" + "\r\n")
        # Initialize individual sampling face count初始化个人采样面数
        count = 0
        
        while(True):

            ret, img = cam.read()                                                                                       #读取图像
            img = cv2.flip(img, -1) # flip video image vertically                                                       #垂直翻转视频图像
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)                                                                #RGB转灰阶
            faces = face_detector.detectMultiScale(gray, 1.3, 5)                                                        #它可以检测出图片中所有的人脸
                                                                                                                        #并将人脸用vector保存各个人脸的坐标、大小（用矩形表示）
                                                                                                                        #第二个参数表示在前后两次相继的扫描中，搜索窗口的比例系数。默认为1.1即每次搜索窗口依次扩大10%
                                                                                                                        #表示构成检测目标的相邻矩形的最小个数(默认为3个)
            for (x,y,w,h) in faces:
                cv2.rectangle(img, (x,y), (x+w,y+h), (255,0,0), 2)                                                      #画矩形
                count += 1                                                                                              #人脸数据采样数加一
                ser.write("count=" + str(count) + "\r\n")

                # Save the captured image into the datasets folder
                cv2.imwrite("dataset/User." + str(face_id) + '.' + str(count) + ".jpg", gray[y:y+h,x:x+w])              #保存灰阶数据

                cv2.imshow('image', img)                                                                                #显示RGB画面

            k = cv2.waitKey(100) & 0xff # Press 'ESC' for exiting video
            if k == 27:
                break
            elif count >= 30: # Take 30 face sample and stop video
                break
                
        cv2.destroyAllWindows()
        print('\n  图像获取完毕！！！')

        
        
        
        print('\n  开始训练数据集，请稍等......')
        print ("\n [INFO] Training faces. It will take a few seconds. Wait ...")
        ser.write("Training faces" + "\r\n")
        faces,ids = getImagesAndLabels(path)                                                                            #获取人脸和标签
        recognizer.train(faces, np.array(ids))                                                                          #训练

        # Save the model into trainer/trainer.yml                                                                       #将模型保存到trainer / trainer.yml中
        recognizer.write('trainer/trainer.yml') # recognizer.save() worked on Mac, but not on Pi                        #identifierr.save（）在Mac上有效，但在Pi上无效

        # Print the numer of faces trained and end program
        print("\n [INFO] {0} faces trained. Exiting Program".format(len(np.unique(ids))))                               #打印经过训练的面部数量
        print('\n  数据集训练完毕！！！')
        
        flag_add=0
        ser.write("Trained" + "\r\n")
        time.sleep(1.5)
        
        
    elif flag_mode==0:                                                                                                  #KNN警觉模式
        
        if KNN_uart_flag==1:
            ser.write("KNN mode" + "\r\n")
            KNN_uart_flag=0
            
        KNN_run_flag = 1                                                                                                #用于确定是否需要清理一下窗口
        REC_uart_flag = 1
        if REC_run_flag:
            cv2.destroyAllWindows() 
            REC_run_flag = 0
    
    
        ret,frame=cam.read()
        fgmask=bs.apply(frame)
        th=cv2.threshold(fgmask.copy(),244,255,cv2.THRESH_BINARY)[1]
        th=cv2.erode(th,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3)),iterations=2)
        
        dilated=cv2.dilate(th,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3)),iterations=2)
        
        image,contours,hier=cv2.findContours(dilated,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        
        for c in contours:              
            if cv2.contourArea(c)>3:
                (x,y,w,h)=cv2.boundingRect(c)
                cv2.rectangle(frame,(x,y),(x+w,y+h),(255,255,0),2)
            if x!=0 and y!=0:
                # print('x',x,'y',y)
                KNN_Warning_new_time = time.time()
                if KNN_Warning_new_time - KNN_Warning_old_time > 1:
                    KNN_Warning_old_time = KNN_Warning_new_time
                    ser.write("Warning!!!!!!" + "\r\n")
                
        cv2.imshow("mog",fgmask)
        cv2.imshow("detection",frame)
        
    elif flag_mode==1:                                                                                              #识别模式
        
        if REC_uart_flag==1:
            ser.write("REC mode" + "\r\n")
            REC_uart_flag=0

        REC_run_flag = 1                                                                                            #用于确定是否需要清理一下窗口
        KNN_uart_flag=1
        if KNN_run_flag:
            cv2.destroyAllWindows() 
            KNN_run_flag = 0
    
        
        ret, img =cam.read()                                                                                        #读取图像
        # img = cv2.flip(img, -1) # Flip vertically                                                                 #垂直翻转视频图像

        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)                                                                 #RGB转灰阶

        faces = faceCascade.detectMultiScale(                                                                       #它可以检测出图片中所有的人脸
            gray,                                                                                                   #并将人脸用vector保存各个人脸的坐标、大小（用矩形表示）
            scaleFactor = 1.2,                                                                                      #第二个参数表示在前后两次相继的扫描中，搜索窗口的比例系数。默认为1.2即每次搜索窗口依次扩大20%
            minNeighbors = 5,                                                                                       #表示构成检测目标的相邻矩形的最小个数(默认为3个)
            minSize = (int(minW), int(minH))                                                                     
            )    


        #记录拍摄的时间
        cv2.putText(img,time.strftime("%Y-%m-%d %H:%M:%S",time.localtime()),(20,20),font,0.8,(255,255,255),1)



        for(x,y,w,h) in faces:

            cv2.rectangle(img, (x,y), (x+w,y+h), (0,255,0), 2)                                                      #画矩形

            id, confidence = recognizer.predict(gray[y:y+h,x:x+w])                                                  #预测

            # Check if confidence is less them 100 ==> "0" is perfect match 
            if (confidence < 100):                                                                                  #检查可信度
                id = names[id]
                confidence = "  {0}%".format(round(100 - confidence))
                email_send_cnt=0
            else :
                id = "unknown"
                confidence = "  {0}%".format(round(100 - confidence))
                email_send_cnt = email_send_cnt + 1
                cv2.imwrite("out.png",img)

            cv2.putText(img, str(id), (x+5,y-5), font, 1, (255,255,255), 2)
            cv2.putText(img, str(confidence), (x+5,y+h-5), font, 1, (255,255,0), 1)  
            
            
            

        NowTimeStamp=time.time()
        #避免在短时间内重复拍摄，设置时间戳
        if NowTimeStamp-OldTimeStamp>15 and email_send_cnt>45:                                                       #距离上一次发送图片的事件间隔要大于15s  ,并且连续4帧检测到陌生人      

            ser.write("Warning!!!!!!" + "\r\n")
            
            picture_file = open('out.png',"rb")
            picture_data = picture_file.read()
            picture_file.close()
            picture = MIMEImage(picture_data)
            picture.add_header('Content-ID', '0')    #正常附件的header是不同的
            msg.attach(picture)
            msg["From"] = Header("xiesicong", "utf-8")
            msg["To"] = Header(receiver, "utf-8")
            msg["Subject"] = Header("Warning！！！！", "utf-8")
            #-----------------将图片作为正文内容添加-------------------
            message = MIMEText("<p>careful!!!!!</p><p>human approach your device</p><img src='cid:0'/>","html","utf-8")    #plain表示纯文本
            msg.attach(message)
            contype = 'application/octet-stream'
            maintype, subtype = contype.split('/', 1)                  
            try:
                #qq必须要用.SMTP_SSL
                #其他服务器try:.SMTP
                smtpObject = smtplib.SMTP_SSL(smtp_server , smtp_port)
                smtpObject.login(sender , password)
                #message.as_string()是将MIMEText对象变成字符串
                smtpObject.sendmail(sender , [receiver] , msg.as_string())
                print ("email发送成功! ")
                smtpObject.quit() 
            except smtplib.SMTPException :
                print ("email发送失败！")
                
                           
            OldTimeStamp=time.time()
            email_send_cnt=0
                    
                    
                    
                    
                    
        cv2.imshow('camera',img)                                                                                    #显示RGB画面





    k = cv2.waitKey(10) & 0xff # Press 'ESC' for exiting video                                                      #按“ ESC”退出视频
    if k == 27:
        break



# Do a bit of cleanup                                                                                               #做一些清理
print("\n [INFO] Exiting Program and cleanup stuff")       
cam.release()                                                                                                       #释放摄像头
cv2.destroyAllWindows()                                                                                             #释放所有窗口








