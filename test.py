#coding=UTF-8
#!/usr/bin/env python3
import argparse
import time
import cv2
import numpy as np
from keras.models import model_from_json
import RPi.GPIO as GPIO
from DFRobot_RaspberryPi_DC_Motor import DFRobot_DC_Motor_IIC as Board



# 設定DF馬達驅動板腳位
board = Board(1, 0x10)    # Select bus 1, set address to 0x10
#PWM_PIN_left = 17
#PWM_PIN_right = 18

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
buzzier = 12
f_TRIG = 23
f_ECHO = 24
b_TRIG = 21
b_ECHO = 22


GPIO.setup(f_TRIG, GPIO.OUT)
GPIO.setup(f_ECHO, GPIO.IN)
GPIO.setup(buzzier,GPIO.OUT)

p = GPIO.PWM(12, 50)
p.start(50)
speed_r = 60
speed_l = 60

# 檢查DF馬達驅動板狀態
def board_detect():
    l = board.detecte()
    print("Board list conform:")
    print(l)

def print_board_status():
    if board.last_operate_status == board.STA_OK:
        print("board status: everything ok")
    elif board.last_operate_status == board.STA_ERR:
        print("board status: unexpected error")
    elif board.last_operate_status == board.STA_ERR_DEVICE_NOT_DETECTED:
        print("board status: device not detected")
    elif board.last_operate_status == board.STA_ERR_PARAMETER:
        print("board status: parameter error, last operate no effective")
    elif board.last_operate_status == board.STA_ERR_SOFT_VERSION:
        print("board status: unsupport board framware version")

IR_LEFT_PIN = 17
IR_MIDDLE_PIN = 27
IR_RIGHT_PIN = 22

DUTY_CYCLE = 40

def get_distance_front():

    GPIO.output(f_TRIG, False)
    time.sleep(2)
    GPIO.output(f_TRIG, True)
    time.sleep(0.00001)
    GPIO.output(f_TRIG, False)

    while GPIO.input(f_ECHO)==0:
        start = time.time()

    while GPIO.input(f_ECHO)==1:
        end = time.time()
    print((end-start)* 17150)
    return (end-start)* 17150

def get_distance_back():
    GPIO.output(b_TRIG, False)
    time.sleep(2)
    GPIO.output(b_TRIG, True)
    time.sleep(0.00001)
    GPIO.output(b_TRIG, False)

    while GPIO.input(b_ECHO)==0:
        start = time.time()

    while GPIO.input(b_ECHO)==1:
        end = time.time()
    print((end-start)* 17150)
    return (end-start)* 17150




def main():
    #檢查DF馬達驅動板狀態
    board_detect()

    while board.begin() != board.STA_OK:    # Board begin and check board status
        print_board_status()
        print("Motor board begin faild")
        time.sleep(2)
    print("Motor board begin success")

    #關閉Encoder功能
    board.set_encoder_disable(board.ALL)

    #設定PWM頻率
    board.set_moter_pwm_frequency(1000)


    # 設定程式參數
    arg_parser = argparse.ArgumentParser(description='軌跡車程式。')
    arg_parser.add_argument(
        '--model-file',
        required=True,
        help='模型架構檔',
    )
    arg_parser.add_argument(
        '--weights-file',
        required=True,
        help='模型參數檔',
    )
    arg_parser.add_argument(
        '--input-width',
        type=int,
        default=48,
        help='模型輸入影像寬度',
    )
    arg_parser.add_argument(
        '--input-height',
        type=int,
        default=48,
        help='模型輸入影像高度',
    )

    # 解讀程式參數
    args = arg_parser.parse_args()
    assert args.input_width > 0 and args.input_height > 0

    # 載入模型
    with open(args.model_file, 'r') as file_model:
        model_desc = file_model.read()
        model = model_from_json(model_desc)

    model.load_weights(args.weights_file)

    # 開啓影片來源
    video_dev = cv2.VideoCapture(0)
    video_dev.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))

    # 初始化 GPIO
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    #GPIO.setup(PWM_PIN_left, GPIO.OUT)
    #GPIO.setup(PWM_PIN_right, GPIO.OUT)
    #pwm1 = GPIO.PWM(PWM_PIN_left, 500)
    #pwm2 = GPIO.PWM(PWM_PIN_right, 500)
    #pwm1.start(0)
    #pwm2.start(0)

    GPIO.setup(IR_RIGHT_PIN, GPIO.IN)  #GPIO 2 -> Left IR out
    GPIO.setup(IR_MIDDLE_PIN, GPIO.IN) #GPIO 3 -> Right IR out
    GPIO.setup(IR_LEFT_PIN, GPIO.IN)   #GPIO 4 -> Right IR out


    def recognize_image():

        # 先丟掉前十張舊的辨識結果
        for i in range(10):
            image = video_dev.read()

        ret, orig_image = video_dev.read()
        assert ret is not None

        # 縮放爲模型輸入的維度、調整數字範圍爲 0～1 之間的數值
        resized_image = cv2.resize(
            orig_image,
            (args.input_width, args.input_height),
        ).astype(np.float32)
        normalized_image = resized_image / 255.0

        # 執行預測
        batch = normalized_image.reshape(1, args.input_height, args.input_width, 3)
        result_onehot = model.predict(batch)
        class_id = np.argmax(result_onehot, axis=1)[0]

        left_score, right_score, stop_score, other_score = result_onehot[0]
        print('預測：%.2f %.2f %.2f %.2f' % (left_score, right_score, stop_score, other_score))

        # print(result_onehot)
        if class_id == 0:
            return 'left'
        elif class_id == 1:
            return 'right'
        elif class_id == 2:
            return 'stop'
        elif class_id == 3:
            return 'other'

    def forward(speed_right,speed_left):
        board.motor_movement([board.M1], board.CCW, speed_right)
        board.motor_movement([board.M2], board.CW, speed_left)

    def head_left(speed_left):
        #pwm1.ChangeDutyCycle(DUTY_CYCLE)
        #pwm2.ChangeDutyCycle(0)
        board.motor_movement([board.M1], board.CCW, 0)
        board.motor_movement([board.M2], board.CW, speed_left)

    def head_right(speed_right):
        #pwm1.ChangeDutyCycle(0)
        #pwm2.ChangeDutyCycle(DUTY_CYCLE)
        board.motor_movement([board.M1], board.CCW, speed_right)
        board.motor_movement([board.M2], board.CW, 0)

    def stop():
        #pwm1.ChangeDutyCycle(0)
        #pwm2.ChangeDutyCycle(0)
        board.motor_movement([board.M1], board.CCW, 0)
        board.motor_movement([board.M2], board.CW, 0)

    def cross_left():
        time.sleep(0.1)

        board.motor_movement([board.M1], board.CCW, 80)
        board.motor_movement([board.M2], board.CW, 80)
        time.sleep(0.4)

        board.motor_movement([board.M1], board.CCW, 0)
        board.motor_movement([board.M2], board.CW, 0)
        time.sleep(0.5)

        board.motor_movement([board.M1], board.CCW, 0)
        board.motor_movement([board.M2], board.CW, 80)

        time.sleep(0.45)

        board.motor_movement([board.M1], board.CCW, 100)
        board.motor_movement([board.M2], board.CW, 100)

        time.sleep(0.3)

        board.motor_movement([board.M1], board.CCW, 0)
        board.motor_movement([board.M2], board.CW, 0)
        time.sleep(0.3)

        board.motor_movement([board.M1], board.CCW, 0)
        board.motor_movement([board.M2], board.CW, 0)

        time.sleep(0.2)

    def cross_right():
        time.sleep(0.1)

        board.motor_movement([board.M1], board.CCW, 80)
        board.motor_movement([board.M2], board.CW, 80)
        time.sleep(0.4)

        board.motor_movement([board.M1], board.CCW, 0)
        board.motor_movement([board.M2], board.CW, 0)
        time.sleep(0.5)

        board.motor_movement([board.M1], board.CCW, 80)
        board.motor_movement([board.M2], board.CW, 0)

        time.sleep(0.45)

        board.motor_movement([board.M1], board.CCW, 100)
        board.motor_movement([board.M2], board.CW, 100)

        time.sleep(0.3)

        board.motor_movement([board.M1], board.CCW, 0)
        board.motor_movement([board.M2], board.CW, 0)
        time.sleep(0.3)

        board.motor_movement([board.M1], board.CCW, 0)
        board.motor_movement([board.M2], board.CW, 0)

        time.sleep(0.2)

        #pwm1.ChangeDutyCycle(0)
        #pwm2.ChangeDutyCycle(100)
        #time.sleep(0.45)

        #pwm1.ChangeDutyCycle(0)
        #pwm2.ChangeDutyCycle(0)
        #time.sleep(1)

        #pwm1.ChangeDutyCycle(100)
        #pwm2.ChangeDutyCycle(100)
        #time.sleep(1)

        #pwm1.ChangeDutyCycle(0)
        #pwm2.ChangeDutyCycle(0)
        #time.sleep(0.5)

    def track_line():
        middle_val = GPIO.input(IR_MIDDLE_PIN)
        left_val = GPIO.input(IR_LEFT_PIN)
        right_val = GPIO.input(IR_RIGHT_PIN)
        print('光感:', left_val, middle_val, right_val)

        if middle_val:
            if left_val and right_val:        # 白白白
                return 'stop'
            elif left_val and not right_val:  # 白白黑
                return 'left'
            elif not left_val and right_val:  # 黑白白
                return 'right'
            else:
                return 'forward'              # 黑白黑
        else:
            if left_val and right_val:        # 白黑白
                return 'stall'
            elif left_val and not right_val:  # 白黑黑
                return 'left'
            elif not left_val and right_val:  # 黑黑白
                return 'right'
            else:                             # 黑黑黑
                return 'stall'


    
        
    
#     try:
#         while True:
#             advice = track_line()
# 
#             if advice == 'left':
#                 print('動作:', '左轉')
#                 head_left()
# 
#             elif advice == 'right':
#                 print('動作:', '右轉')
#                 head_right()
# 
#             elif advice == 'stop':
#                 print('動作:', '停止')
#                 stop()
# 
#                 sign = recognize_image()
# 
#                 if sign == 'left':
#                     print('影像:', '左轉標誌')
#                     cross_left()
# 
#                 elif sign == 'right':
#                     print('影像:', '右轉標誌')
#                     cross_right()
# 
#                 elif sign == 'stop':
#                    print('影像:', '停止標誌')
# 
#                 elif sign == 'other':
#                     print('影像:', '無標誌')
# 
#             elif advice == 'forward':
#                 print('動作:', '前進')
#                 forward()
# 
#             elif advice == 'stall':
#                 print('動作:', '前進')
#                 forward()
# 
#             print()
# 
#     except KeyboardInterrupt:
#         print('使用者中斷')

    try:
        while True:
            sign = recognize_image()
            GPIO.output(f_TRIG, False)
            time.sleep(1)
            GPIO.output(f_TRIG, True)
            time.sleep(0.00001)
            GPIO.output(f_TRIG, False)

            while GPIO.input(f_ECHO)==0:
                start = time.time()

            while GPIO.input(f_ECHO)==1:
                end = time.time()

            f_distance = get_distance_front()
            b_distance = get_distance_back()
    
            if f_distance >20:
                board.motor_movement([board.M1], board.CCW, 60)
                board.motor_movement([board.M2], board.CW, 60)
                speed_r = 60
                speed_l = 60
            while f_distance <=20 and f_distance >5:
                GPIO.output(f_TRIG, False)
                time.sleep(1)
                GPIO.output(f_TRIG, True)
                time.sleep(0.00001)
                GPIO.output(f_TRIG, False)

                while GPIO.input(f_ECHO)==0:
                    start = time.time()

                while GPIO.input(f_ECHO)==1:
                    end = time.time()

                f_distance = (end-start)* 17150
                speed_r -= 5
                speed_l -= 5
                if sign == 'forward':
                    forward(speed_r,speed_l)
                elif sign == 'left':
                    head_left(speed_l)
                elif sign == 'rihgt':
                    head_right(speed_r)
            
            if f_distance <=5:
                stop()
                time.sleep(0.1)

            if b_distance <=10 and b_distance >5:
                GPIO.output(buzzier.True)
            elif b_distance <=5:
                GPIO.output(buzzier.True)
                time.sleep(1)
                stop()
                GPIO.output(buzzier.False)
            else:
                GPIO.output(buzzier.False)

    except KeyboardInterrupt:
        print("stop")


    # 終止馬達
    stop()
    GPIO.cleanup()
    # 終止影像裝置
    video_dev.release()


if __name__  == '__main__':
    main()



