#!/usr/bin/env python
# coding: utf-8

from drone import *
from ljmq import *
from vision import *
import time
import cv2
import os


drone = Drone()
#message = Ljmq("S1", "Q0")
drone.now_wp=0
v = Vision()
cap=cv2.VideoCapture(2)

mode = "MISSION"
targets=[]
t = 0
loiter_time = 0

signal.signal(signal.SIGINT, quit)   # 设置退出函数                             
signal.signal(signal.SIGTERM, quit)
pwmopen(1)
print("BLUE")
pwmduty(1,1800)
time.sleep(2)
pwmduty(1,1000)
time.sleep(1.5)
pwmopen(2)
print("RED")
pwmduty(2,1800)
time.sleep(2)
pwmduty(2,1000)

# 打开避障
#drone.obs_avoidance(1)
print("开始执行任务")


while True:
# 到达某航点后认为自己没电
    #if drone.now_wp == 7: break
    print("flying, now_wp:", drone.now_wp)
    #time.sleep(0.5)

    # print("read frame:", t)
    t += 1
    time.sleep(0.1)  # 10hz检测频率
    ret,img=cap.read()
    qd = v.Calculate_QD(img)
    print(qd)
    cv2.imwrite("picture/raw/" + str(t) + ".jpg", img)
    try:  # 如果没有正常读到帧
        img_blur = cv2.GaussianBlur(img, (7,7), 1, 0)  # 高斯滤波
    except: continue

    threshold = {
                "red":(np.array([0,70,90]), np.array([15,220,230])),
                "blue":(np.array([98, 43, 46]), np.array([120, 255, 255])),
                #"yellow":(np.array([26,43,46]), np.array([34,255,255]))
                }

    masks = v.color_area(img_blur, ["red", "blue"], thresholds=threshold)  # 获取掩膜
    cnts_red = v.contour(masks["red"])  # [cnt1,cnt2,cnt3,cnt4,cnt5]
    cnts_blue = v.contour(masks["blue"])  # 提取该帧所有轮廓

    min_s = 80*80  # 轮廓最小面积
    s = min_s
    for cnt in cnts_red:  # 获取所有轮廓的形状、box、中心
        out = v.shape(cnt)
        x,y,w,h = out[1]  # box
        shape = out[0]
        # if shape == "unidentified": continue
        if abs(0.5*((w-h)/(w+h))) < 0.12:  # 限制box纵横比         
            # 找出面积最大的轮廓
            if s < w*h:
                s=w*h
                target=(out, cnt, "red")  # 轮廓信息、轮廓矩阵、颜色
    for cnt in cnts_blue: 
        out = v.shape(cnt)
        x,y,w,h = out[1]
        shape = out[0]
        # if shape == "unidentified": continue
        if abs(0.5*((w-h)/(w+h))) < 0.12:       
            if s < w*h:
                s=w*h
                target=(out, cnt, "blue")

    # print("start thre, loiter_time:", loiter_time)
    if s > min_s:  # 发现目标
        print("Find target, now mode:", mode)
        #----------
        x,y,w,h=target[0][1]
        centre = target[0][2]
        now_time = str(time.asctime(time.localtime(time.time())))
        color = target[2]
        cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.putText(img,now_time, (360,450), v.font, 0.5, (255,255,255), 2)
        #----------
        cv2.imwrite("picture/out/" + str(t) + ".jpg", img)

        if mode == "MISSION":  # 航线模式下，进入悬停模式
            mode = "AUTO.LOITER"
            print("set mode LOITER")
            time.sleep(0.5)
            drone.set_flymode("AUTO.LOITER")
            loiter_time = 0  # 重置悬停时长检测
            time.sleep(5)
            continue
        elif mode == "AUTO.LOITER":
            print("LOITER check:", loiter_time)
            if loiter_time < 30:  # 悬停3s拍照
                targets.append(target)
                time.sleep(0.1)
                loiter_time += 1
                # continue
    # No Target
    elif mode == "AUTO.LOITER": 
        print("LOITER but No target:",str(t))
        loiter_time += 1
    else: print("No target:",str(t))
    
    if mode == "AUTO.LOITER" and loiter_time >= 30:
        print("check data")
        if len(targets)>10:  # 3s内有30%照片内存在目标，则认为识别准确
            print("send a massage")
            if target[2] == "red":
                pwmduty(2,1800)
                #message.push_color(color="red")
                print("red")
            else:
                pwmduty(1,1800)
                #message.push_color(color="blue")
                print("blue")
            # 自行补充pwm1、2的判断和控制

            time.sleep(2)
                    
        else: print("data error")
        targets=[]
        print("set mode MISSION")  # 无论是否打开抓手，都继续航线
        drone.set_flymode("AUTO.MISSION")
        mode = "MISSION"
        loiter_time = 0  # 重置悬停时长检测
        time.sleep(2)


drone.set_flymode("AUTO.LOITER")

# 启动通讯并发送meet指令
message.start()
message.push(cmd="meet")
while message.Q0["result"] != 1:
    time.sleep(1)

# 请自行实现飞往小车上方并悬停的功能
