   #!/usr/bin/env python
# coding: utf-8

from drone import *
from ljmq import *
from vision import *
import time
led=Ljmq("S1","Q0") #实例化LED灯通讯
drone=Drone()
v=Vision()
t=0
targets=[]
cap=cv2.VideoCapture(2)
mode="MISSION"
loiter_time = 0
drone.now_wp =0
s_f=0
s_t=0

signal.signal(signal.SIGINT, quit)   # 设置退出函数
signal.signal(signal.SIGTERM, quit)

pwmclose(2)#防止pwm已经打开
pwmclose(1)


pwmopen(2)#初始化输出
pwmopen(1)

pwmduty(1, 1900)#打开抓手1
pwmduty(2, 1900)#打开抓手1
time.sleep(3)
pwmduty(1, 1000)
time.sleep(2)
pwmduty(2, 1000)

#当飞机未起飞时阻塞在该循环
while True:
    if drone.now_wp==0:
        time.sleep(0.5)
        break

drone.obs_avoidance(1)
print("飞行器已起飞，开启避障")


while True:
    if drone.now_wp==2:
        time.sleep(0.5)
        break

drone.obs_avoidance(0)
print("已完成避障任务，关闭避障")

#time.ctime()函数为打印当前日期+时间，可以代替time.timeasc（time.localtime(time.time())）

#视觉检测循环
while True:
    t += 1
    time.sleep(0.1)  # 10hz检测频率
    ret,img=cap.read()
    cv2.imwrite("picture/raw/" + str(t) + ".jpg", img)
    try:  # 如果没有正常读到帧
        img_blur = cv2.GaussianBlur(img, (7,7), 1, 0)  # 高斯滤波
    except: continue

    threshold = {
                "red":(np.array([2,80,50]), np.array([15,220,230])),
                "blue":(np.array([100, 80, 50]), np.array([120, 255, 255])),
                "yellow":(np.array([26,43,46]), np.array([34,255,255]))
                }

    masks = v.color_area(img_blur, ["red", "blue"], thresholds=threshold)  # 获取掩膜
    cnts_red = v.contour(masks["red"])  # [cnt1,cnt2,cnt3,cnt4,cnt5]
    cnts_blue = v.contour(masks["blue"])  # 提取该帧所有轮廓

    min_s = 150*150  # 轮廓最小面积
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
        color = target[2]
        cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.putText(img, color+"-"+shape, centre, v.font, 0.5, (255,255,255), 2)
        #----------
        cv2.imwrite("out/" + str(t) + ".jpg", img)
        if mode == "MISSION" and s_f > 2:  # 航线模式下，进入悬停模式
            mode = "AUTO.LOITER"
            s_f = 0
            print("set mode LOITER")
            # time.sleep(0.5)
            drone.set_flymode("AUTO.LOITER")
            loiter_time = 0  # 重置悬停时长检测
            time.sleep(2)#等待飞机稳定悬停
            continue
        elif mode == "MISSION" and s_f <=2:
            if s_t == (t-1):
                s_f += 1
            else:s_f = 0
            s_t = t
            continue
        elif mode == "AUTO.LOITER":
            print("LOITER check:", loiter_time)
            if loiter_time < 30:  # 悬停3s拍照
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
        if len(targets)>10:  # 3s内有30%照片内存在目标，则认为识别准确
            print("Open")
            if targets[0][2]=="red":
                print("发现红色")
                pwmduty(1,1900)
                print("打开舵机1")
                led.push_color("red")
                time.sleep(3)

            elif targets[0][2]=="blue":
                print("发现蓝色")
                pwmduty(2,1900)
                print("打开舵机2")
                led.push_color("blue")
                time.sleep(3)

            time.sleep(2)
                    
        else: print("data error")
        targets=[]
        print("set mode MISSION")  # 无论是否打开抓手，都继续航线
        drone.set_flymode("AUTO.MISSION")
        mode = "MISSION"
        loiter_time = 0  # 重置悬停时长检测
        time.sleep(2)

drone.set_flymode("AUTO.MISSION")
