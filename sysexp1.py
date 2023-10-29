# -*- coding: utf-8 -*-
"""
Created on Sat Sep 17 20:13:47 2022

@author: Hak
"""

from tello_vib import tello_vib
import time
import numpy as np

# Log
is_log = True
is_rec = True

# Tello 정의
Tello_1 = tello_vib(is_log,is_rec)
Tello = Tello_1.tello

# check frame
frame_tmp = np.zeros((960,720,3))

#%%
# PID log
kp,kd,ki = 0,0,0
Tello_1.kp, Tello_1.kd, Tello_1.ki = kp,kd,ki
time_prev = time.time() - 1

#####################################
TurnLeft = False
TurnRight = False
LandingSign = False
YawingTime = 5     # 20    22
#####################################

LandingSign = False
while not Tello_1.stop :
    #%% control
    frame = Tello_1.frame_read.frame
    text = np.array([])
    
    # check frame
    if np.sum(frame) == np.sum(frame_tmp):
        continue
    else:
        frame_tmp = frame
    
    move_right, move_up, move_front, yaw = 0, 0, 0 ,0
    x,y,w,h = 0,0,0,0

    if Tello_1.tracking:
        ##################### drone control #########################
        """
        Drone velocities between -100~100
        define --> vel = [left_right_velocity, front_back_velocity, up_down_velocity, yaw_velocity]
        
        control by --> update(vel)
        
        front_back_velocity --> + : front, - : back
        left_right_velocity --> + : right, - : left
        up_down_velocity    --> + : up,    - : down
        yaw_velocity        --> + : cw,    - : ccw
        
        """
        
        # class_no 1 : CAU, 2: Left, 3: Right
        x,y,w,h,label,conf,class_no = Tello_1.detect(frame, Tello_1.model)     # 타겟의 좌표(x,y), 크기(w,h), 정확도(conf), 종류(class_no)
        
        ##################################
        #### 학생들이 수정할 부분 시작 ####
        ##################################
        
        # x : -480 ~ 480
        # y : -360 ~ 360
        ################################################
        # 카메라 기준 좌상단 점의 x, y가 0, 0으로 시작해서 아래로 갈수록 숫자가 높아짐
        # 따라서 x, y 좌표의 중심에서 threshold를 지정, 범위 내에 드론이 올 수 있도록 제어 코드 작성 필요
        ################################################
        
        # 랜딩 사인을 인식하지 않을 시 드론 기동
        if class_no == 0:
            if TurnLeft == True:
                move_front = 0
                move_right = 0
                move_up = 0
                yaw = -80  # -80
            elif TurnRight == True:
                move_front = 0
                move_right = 0
                move_up = 0
                yaw = 80   # 80
            # move_front = 22
            # move_up = 22
            # 아무것도 인식 안할 시 적당히 앞으로 가도록 명령
        else:
            TurnLeft = False
            TurnRight = False
            yaw = 0
            # 어떤 표지라도 인식할 시 아래 코드 실행
            move_front = 35   # 22
            # 앞으로 천천히 이동
            if y < -70:
                move_up = -20
            elif y > 70:
                move_up = 20
            # 타겟의 높이가 -100~100 이내의 범위에 없을 시 고도 조정
                
            if x > 70:
                move_right = 20
                # 오른쪽으로
            elif x < -70:
                move_right = -20
                # 왼쪽으로
            # 타켓의 중심이 -100~100 이내의 범위에 없을 시 드론 roll 기동
            
            if w >= 250:        # 270
                # 인식 후 범위 내에 들어올 시 yaw 기동
                # move_front = 5        # 0
                # move_right = 0
                if class_no == 2:
                    # 왼쪽 화살표 인식할 시
                    TurnLeft = True
                elif class_no == 3:
                    # 오른쪽 화살표 인식할 시
                    TurnRight = True
                elif class_no == 1:
                    LandingSign = True
                    
        if TurnLeft == True:
            # TurnLeft가 true로 변환 시 yaw를 제외한 모든 동작 0으로 반환
            move_front = 15    # 0
            move_right = -8    # 0
            move_up = 0
            yaw = -60   # -40
            YawingTime -= 1
            if YawingTime < 0:
                YawingTime = 5
                # while문이 20번 돌면 TurnLeft False로 반환
                
        elif TurnRight == True:
            # TurnRight가 true로 변환 시 yaw를 제외한 모든 동작 0으로 반환
            move_front = 15    # 0
            move_right = 8     # 0
            move_up = 0
            yaw = 60    # 40
            YawingTime -= 1
            if YawingTime < 0:
                YawingTime = 5
                # while문이 20번 돌면 TurnRight False로 반환
        
        elif LandingSign == True:
            # 범위에 해당, CAU 마크 인식 시 LandingSign True로 전환
            # 드론의 모든 움직임 정지, 랜딩 함수 인가
            move_right, move_up, move_front, yaw = 0, 0, -3 ,0
            Tello_1.land()
        

        vel = [move_right,move_front,move_up,yaw]
        Tello_1.update(vel)
        
        ##################################
        ##### 학생들이 수정할 부분 끝 #####
        ##################################
        
        # Plot
        Tello_1.plot_tracking(x)
    
    else: # Show plot
        Tello_1.plot_not_tracking()

    text = np.array([f"move_right : {move_right}", f"move_front : {move_front}",
                      f"move_up: {move_up}", f"yaw : {yaw}"])
    
    frame = Tello_1.write_txt(frame, text)
    Tello_1.log(x,y,w,h,move_right,move_front,move_up,yaw)
    
Tello_1.end()