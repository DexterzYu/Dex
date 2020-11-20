#config:utf-8
from drone import *

from vision import *
from ljmq import *
import time

#led=Ljmq("S","A22")
#message=Ljmq("S","Q17368")

drone=Drone()
t=0
targets=[]
drone.now_wp=0
loiter_time=0
mode="MISSION"
v=Vision()
cap=VideoCapture(2)
s_f,s_t=0,0

signal.signal(signal.SIGINT,quit)
signal.signal(signal.SIGTERM,quit)
pwmopen(1,duty=1000)
pwmopen(2,duty=1000)
time.sleep(2)
pwmopen(1,duty=1800)
time.sleep(2)
pwmopen(2,duty=1800)

drone.obs_avoudance(0)
drone.terrial(0)

print("all ok")

print("car start")
'''
message.start()
message.push(cmd="start")

while message.role2["result"]!=1:
    time.sleep(0.1)

messgae.push()
'''
while drone.now_wp<1:
    time.sleep(2)

for i in rangle(2):
    led.push_color("yellow")
    time.sleep(2)

while drone.now_wp<2:
    time.sleep(0.5)

drone.obs_avoidance(1)
print("flying,open")

while drone.now_wp<:
    time.sleep(0.5)
    drone.set_flymode("AUTO.MISSION")

drone.obs_avoidance(0)

while True:
    t+=1
    time.sleep(0.05)
    ret,img=cap.read()
    cv2.imwrite("/home/spot/usr/picture/raw"+str(t)+".jpg",img)
    try:
        img_blur=cv2.GaussianBlur(img,(7,7),1,0)
    except:continue

    threshold={
               "red":(np.array([156,43,46]),np.array([186,255,255])),
               "blue":(np.array([110,43,46]),np.array([124,255,255])),
               "yellow":(np.array([15,43,46]),np.array([25,255,255]))
    }

    masks=v.color_area(img_blur,["red","blue","yellow"],thresholds=threshold)
    cnts_red=(masks["red"])
    cnts_blue=(masks["blue"])
    cnts_yellow=(masks["yellow"])

    min_s=100*100
    s=min_s

    for cnt in cnts_red:
        out=v.shape(cnt)
        x,y,w,h=out[1]
        shape=out[0]
        if abs(0.5*((w-h)/(w+h)))<0.12:
            if s<w*h:
                s=w*h
                target=(out,cnt,"red")

    for cnt in cnts_blue:
        out=v.shape(cnt)
        x,y,w,h=out[1]
        shape=out[0]
        if abs(0.5*((w-h)/(w+h)))<0.12:
            if s<w*h:
                s=w*h
                target=(out,cnt,"blue")
    
    for cnt in cnts_yellow:
        out=v.shape(cnt)
        x,y,w,h=out[1]
        shape=out[0]
        if abs(0.5*((w-h)/(w+h)))<0.12:
            if s<w*h:
                s=w*h
                target=(out,cnt,"yellow")

    if s>min_s:
        print("find target now mode:",mode)
        x,y,w,h=target[0][1]
        centre=target[0][2]
        color=target[2]
        cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
        cv2.putText(img,color+"-"+shape,centre,v.font,0.5,(255,255,255),2)
        cv2.imwrite("/home/spot/usr/picture/out/"+str(t)+".jpg",img)

        if mode=="MISSION" and s_f>2:
            mode="AUTO.LOITER"
            s_f=0
            print("SET MODE LOITER")
            drone.set_flymode("AUTO.LOITER")
            loiter_time=0
            time.sleep(2)
            continue

        elif mode=="MISSION" and s_f<=2:
            if s_t==(t-1):
                s_f+=1
            else:s_f=0
            s_t=t
            continue

        elif mode=="AUTO.LOITER":
            print("loiter check:",loiter_time)
            if loiter_time<30:
                targets.append(target)
                time.sleep(0.1)
                loiter_time+=1

    elif mode=="AUTO.LOITER":
        print("loiter but no target:",str(t))
        loiter_time+=1
    else:print("no target:",str(t))

    if mode =="AUTO.LOITER"  and loiter_time>10:
        print("data check")
        if len(targets)>3:
            print("open")
            if targets[0][2]=="red":
                print("find red")
                pwmopen(1,duty=1000)
                print("open1")
                led.push_color("red")
                time.sleep(2)

            if targets[0][2]=="blue":
                print("find blue")
                pwmopen(2,duty=1000)
                print("open2")
                led.push_color("blue")
                time.sleep(2)

            time.sleep(2)

        else:print("data error")
        targets=[]
        print("set mode mission")
        drone.set_flymode("AUTO.MISSION")
        mode="MISSION"
        loiter_time=0
        time.sleep(2)








        





drone.set_flymode("AUTO.LOITER")
cap.release()

message.push(cmd="meet")

while message.role2["result"]!=1:
    time.sleep(1)

meet_lat,meet_long,yaw=message.role2["args"]["latitude"],message.role2["args"]["longitude"],message.role2["args"]["yaw"]

message.push()

drone.set_next_point(meet_lat,meet_long,yaw=yaw)
drone.set_flymode("AUTO.LOITER")

message.push(cmd="land")

while message.role2["result"]!=1:
    time.sleep(1)
message.push()

drone.land(meet_lat,meet_long,yaw=yaw)

message.push(cmd="wait_charge")

while message.role2["result"]!=1:
    time.sleep(1)
message.push()

message.push(cmd="charge",args={"status":"False"})

while message.role2["result"]!=1:
    if message.role2["result"]==2:
        message.push(cmd="charge",args={"status":"True"})
    time.sleep(1)

message.push()
message.push(cmd="takeoff",args={"status":0})

while message.role2["result"]!=1:
    time.sleep(1)

drone.takeoff()

message.push(cmd="takeoff",args={"status":1})
drone.set_flymode("AUTO.MISSION")
time.sleep(5)

message.push()


while drone.now_wp<x:
    time.sleep(1)

drone.terrial(1)
while drone.now_wp<x:
    time.sleep(1)
drone.terrial(0)
while drone.now_wp<x:
    time.sleep(1)
drone.obs_avoidance(1)

while drone.now_wp<x:
    time.sleep(0.5)
    drone.set_flymode("AUTO.MISSION")
drone.obs_avoidance(0)
drone.set_flymode("AUTO.LOITER")
message.push(cmd="meet")

while message.role2["result"]!=1:
    time.sleep(1)

meet_lat,meet_long,yaw=message.role2["args"]["latitude"],message.role2["args"]["longitude"],message.role2["args"]["yaw"]

message.push()

drone.set_next_point(meet_lat,meet_long,yaw=yaw)
drone.set_flymode("AUTO.LOITER")

message.push(cmd="land")

while message.role2["result"]!=1:
    time.sleep(1)
message.push()

drone.land(meet_lat,meet_long,yaw=yaw)

message.push(cmd="return")

time.sleep(1)

while message.role2["result"]!=1:
    time.sleep(1)

message.push()
'''
