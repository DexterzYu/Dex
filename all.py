#!/usr/bin/env python
# coding: utf-8

from drone import *
from ljmq import *
from vision import *
import time
import os
import signal
import cv2

led=Ljmq("S1","Q0") 
drone=Drone()
v=Vision()
t=0
targets=[]

cap=cv2.VideoCapture(2)
mode="MISSION"

loiter_time = 0
drone.now_wp =0
s_t=0
s_f=0


signal.signal(signal.SIGINT, quit)   
signal.signal(signal.SIGTERM, quit)

pwmclose(2)
pwmclose(1)

pwmopen(2)
pwmopen(1)


pwmduty(1, 1900)
print("red")
time.sleep(2)
pwmduty(1, 1000)
time.sleep(3)
pwmduty(2, 1900)
print("blue")
time.sleep(2)
pwmduty(2, 1000)


while True:
    if drone.now_wp>=1:
        time.sleep(0.5)
        break

drone.obs_avoidance(1)
print("open")


while True:
    if drone.now_wp>=2:
        time.sleep(0.5)
        break

drone.obs_avoidance(0)
print("close")


while True:

    if drone.now_wp >= 8: break 

    t += 1
    time.sleep(0.1)  
    ret,img=cap.read()
    qd=v.Calculate_QD(img)
    print(qd)
    cv2.imwrite("/home/spot/demo/picture/raw/" + str(t) + ".jpg", img)
    try:  
        img_blur = cv2.GaussianBlur(img, (7,7), 1, 0) 
    except: continue


    
    threshold = {
                "red":(np.array([0,70,70]), np.array([15,230,230])),
                "blue":(np.array([98, 70, 70]), np.array([120, 230, 230])),
                "yellow":(np.array([15,70,70]), np.array([25,230,230]))
                }

    masks = v.color_area(img_blur, ["red", "blue","yellow"], thresholds=threshold)  
    cnts_red = v.contour(masks["red"])  
    cnts_blue = v.contour(masks["blue"])
    cnts_yellow = v.contour(masks["yellow"])  

    min_s = 150*150  
    s = min_s
    for cnt in cnts_red:  
        out = v.shape(cnt)
        x,y,w,h = out[1]  # box
        shape = out[0]

        if abs(0.5*((w-h)/(w+h))) < 0.12:      
            
            if s < w*h:
                s=w*h
                target=(out, cnt, "red") 
    for cnt in cnts_blue: 
        out = v.shape(cnt)
        x,y,w,h = out[1]
        shape = out[0]
       
        if abs(0.5*((w-h)/(w+h))) < 0.12:       
            if s < w*h:
                s=w*h
                target=(out, cnt, "blue")
    for cnt in cnts_yellow: 
        out = v.shape(cnt)
        x,y,w,h = out[1]
        shape = out[0]
        
        if abs(0.5*((w-h)/(w+h))) < 0.12:       
            if s < w*h:
                s=w*h
                target=(out, cnt, "yellow")

    # print("start thre, loiter_time:", loiter_time)
    if s > min_s: 
        print("Find target, now mode:", mode)
        #----------
        x,y,w,h=target[0][1]
        centre = target[0][2]
        color = target[2]
        now_time = str(time.ctime())
        cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.putText(img, color+"-"+shape, centre, v.font, 0.5, (255,255,255), 2)
        cv2.putText(img,now_time,(360,450),v.font, 0.5, (255,255,255), 2)
        cv2.imwrite("/home/spot/demo/picture/out/" + str(t) + ".jpg", img)
        if mode == "MISSION" and s_f > 2: 
            mode = "AUTO.LOITER"
            s_f = 0
            print("set mode LOITER")
            # time.sleep(0.5)
            drone.set_flymode("AUTO.LOITER")
            loiter_time = 0  
            time.sleep(2)
            continue

        elif mode == "MISSION" and s_f <=2:
            if s_t == (t-1):
                s_f += 1
            else:s_f = 0
            s_t = t
            continue

        elif mode == "AUTO.LOITER":
            print("LOITER check:", loiter_time)
            if loiter_time < 30:  
                targets.append(target)
                time.sleep(0.1)
                loiter_time += 1

    # No Target
    elif mode == "AUTO.LOITER": 
        print("LOITER but No target:",str(t))
        loiter_time += 1
    else: print("No target:",str(t))
    
    if mode == "AUTO.LOITER" and loiter_time >= 30:
        print("check data")
        if len(targets)>10:  
            print("Open")
            if targets[0][2]=="red":
                print("red")
                pwmduty(1,1900)
                print("open1")
                led.push_color("red")
                time.sleep(3)

            elif targets[0][2]=="blue":
                print("bule")
                pwmduty(2,1900)
                print("open2")
                led.push_color("blue")
                time.sleep(3)
                
            elif targets[0][2]=="yellow":
                print("yellow")
                pwmduty(1,1900)
                print("open1")
                led.push_color("yellow")
                time.sleep(3)

            time.sleep(2)
                    
        else: print("data error")
        targets=[]
        print("set mode MISSION")  
        drone.set_flymode("AUTO.MISSION")
        mode = "MISSION"
        loiter_time = 0  
        time.sleep(2)


drone.set_flymode("AUTO.MISSION")

