import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# 馬達腳位設定（樹莓派實體腳位編號 BOARD）
Motor_R1_Pin = 16
Motor_R2_Pin = 18
Motor_L1_Pin = 11
Motor_L2_Pin = 13
t = 0.65
dc = 50

GPIO.setmode(GPIO.BOARD)
GPIO.setup(Motor_R1_Pin, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Motor_R2_Pin, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Motor_L1_Pin, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Motor_L2_Pin, GPIO.OUT, initial=GPIO.LOW)

# 初始化四個 PWM 控制器（頻率 500Hz）
pwm_r1 = GPIO.PWM(Motor_R1_Pin, 500)
pwm_r2 = GPIO.PWM(Motor_R2_Pin, 500)
pwm_l1 = GPIO.PWM(Motor_L1_Pin, 500)
pwm_l2 = GPIO.PWM(Motor_L2_Pin, 500)
pwm_r1.start(0)
pwm_r2.start(0)
pwm_l1.start(0)
pwm_l2.start(0)


def stop():
    pwm_r1.ChangeDutyCycle(0)
    pwm_r2.ChangeDutyCycle(0)
    pwm_l1.ChangeDutyCycle(0)
    pwm_l2.ChangeDutyCycle(0)

def forward():
    #stack.append("w")
    pwm_r1.ChangeDutyCycle(50)
    pwm_r2.ChangeDutyCycle(0)
    pwm_l1.ChangeDutyCycle(50)
    pwm_l2.ChangeDutyCycle(0)
    time.sleep(0.2)
    stop()
    time.sleep(1)

def backward():
    #stack.append("s")
    pwm_r1.ChangeDutyCycle(0)
    pwm_r2.ChangeDutyCycle(50)
    pwm_l1.ChangeDutyCycle(0)
    pwm_l2.ChangeDutyCycle(50)
    time.sleep(0.2)
    stop()
    time.sleep(1)

def turnLeft():
    #stack.append("d")
    pwm_r1.ChangeDutyCycle(70)
    pwm_r2.ChangeDutyCycle(0)
    pwm_l1.ChangeDutyCycle(0)
    pwm_l2.ChangeDutyCycle(0)
    time.sleep(0.2)
    stop()
    time.sleep(1)

def turnRight():
    #stack.append("a")
    pwm_r1.ChangeDutyCycle(0)
    pwm_r2.ChangeDutyCycle(0)
    pwm_l1.ChangeDutyCycle(70)
    pwm_l2.ChangeDutyCycle(0)
    time.sleep(0.2)
    stop()
    time.sleep(1)

def cleanup():
    stop()
    pwm_r1.stop()
    pwm_r2.stop()
    pwm_l1.stop()
    pwm_l2.stop()
    GPIO.cleanup()

width = 640
length = 480
center = width / 2
webcam = cv2.VideoCapture(0)

# width = 320
# length = 240
# center = width / 2
# webcam = cv2.VideoCapture(0, cv2.CAP_DSHOW)
# webcam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
# webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, length)
webcam.set(cv2.CAP_PROP_BUFFERSIZE,1)

try:
    while 1:
        ret, image = webcam.read()
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        blur_gray = cv2.GaussianBlur(gray,(3, 3), 0)
        # 邊緣偵測
        canny = cv2.Canny(blur_gray, 50, 50)
        cv2.imshow("canny", canny)
        # cv2.waitKey(0)
        black_img = np.zeros(image.shape, np.uint8)
        indices = np.where(canny != [0])
        coordinates = zip(indices[1], indices[0])
        coordinates = (tuple(coordinates))
        # 計算所有邊緣點的 X 平均值
        sum = 0
        for point in coordinates:
            image = cv2.circle(
                black_img, (point[0], point[1]), radius=0, color=(0, 0, 255), thickness=-1
            )
            sum += point[0]
        # print(webcam.get(cv2.CAP_PROP_FRAME_WIDTH), end=" ,") 
        # print(webcam.get(cv2.CAP_PROP_FRAME_HEIGHT))
        sum /= len(coordinates)
        offset = sum - center
        print(offset) 
        if(offset > 80):
            print("turning right")
            turnRight()
        elif(offset < -80):
            print("turning left")
            turnLeft()
        else:
            print("going forward")
            forward()


        # cv2.imshow("canny", canny)
        # cv2.imshow("black_img", black_img)
        # cv2.imshow("blur_gray", blur_gray)
        
        if cv2.waitKey(100) & 0xFF == ord("q"):
            break
except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()
    webcam.release()
    cv2.destroyAllWindows()

