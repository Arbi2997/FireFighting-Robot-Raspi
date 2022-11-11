import cv2
import numpy as np
import os
import time
os.system('sudo pigpiod')
time.sleep(1)
import pigpio
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

#[Motor BLDC]##################################################################
ESC = 18
pi = pigpio.pi()
pi.set_servo_pulsewidth(ESC, 0)
TopSpeed = 1500

#[Motor Driver]################################################################
Ena1, In1, In2 = 4, 21, 20
Enb1, In3, In4 = 17, 17, 12

Ena2, In5, In6 = 27, 26, 29
Enb2, In7, In8 = 22, 13, 6

GPIO.setup(In1, GPIO.OUT)
GPIO.setup(In2, GPIO.OUT)
GPIO.setup(Ena1, GPIO.OUT)
pwm1 = GPIO.PWM(Ena1, 100)
pwm1.start(0)

GPIO.setup(In3, GPIO.OUT)
GPIO.setup(In4, GPIO.OUT)
GPIO.setup(Ena1, GPIO.OUT)
pwm2 = GPIO.PWM(Ena1, 100)
pwm2.start(0)

GPIO.setup(In5, GPIO.OUT)
GPIO.setup(In6, GPIO.OUT)
GPIO.setup(Ena2, GPIO.OUT)
pwm3 = GPIO.PWM(Ena2, 100)
pwm3.start(0)

GPIO.setup(In7, GPIO.OUT)
GPIO.setup(In8, GPIO.OUT)
GPIO.setup(Ena2, GPIO.OUT)
pwm4 = GPIO.PWM(Ena2, 100)
pwm4.start(0)

#[Camera]######################################################################
cap = cv2.VideoCapture(0)
lower = [10,50,50]
upper = [15,255,255]

lower = np.array(lower, dtype='uint8')
upper = np.array(upper, dtype='uint8')

Jarak = 70
Lebar = 30

def FocalLength(measured_distance, real_width, width_in_rf_frame):
    length = (width_in_rf_frame * measured_distance) / real_width
    return length
def Distance_finder(Focal_length, real_face_width, face_width_in_frame):
    distance = (real_face_width * Focal_length) / face_width_in_frame
    return distance

def Object_data(image):
    object_width = 0
    blur = cv2.GaussianBlur(image)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    for contour in contours:
        x = 12579
        if cv2.contourArea(contour)>x:
            x,y,w,h = cv2.boundingRect(contour)
            cv2.rectangle(image, (x,y), (x+w,y+h), (0,255,0), 2)
            object_width = w

    return object_width

capture = cv2.imread('candle.jpg')
find_captures = Object_data(capture)
Object_length = FocalLength(Jarak, Lebar, find_captures)

while True:
    ret, frame = cap.read()
    frame = cv2.resize(frame, (1400,1000), interpolation=cv2.INTER_AREA)
    object_widths = Object_data(frame)

    if object_widths != 0:
        Distance = Distance_finder(Object_length, Jarak, object_widths)
        cv2.putText(frame, f"Distance: {round(Distance, 2)} MM", (30,35), cv2.FONT_HERSHEY_COMPLEX, 1, (255,255,255), 2)

        if Distance <= 300:
            GPIO.output(In1, GPIO.LOW)
            GPIO.output(In2, GPIO.LOW)
            pwm1.ChangeDutyCycle(10)

            GPIO.output(In3, GPIO.LOW)
            GPIO.output(In4, GPIO.LOW)
            pwm2.ChangeDutyCycle(10)

            GPIO.output(In5, GPIO.LOW)
            GPIO.output(In6, GPIO.LOW)
            pwm3.ChangeDutyCycle(10)

            GPIO.output(In7, GPIO.LOW)
            GPIO.output(In8, GPIO.LOW)
            pwm4.ChangeDutyCycle(10)

            pi.set_servo_pulsewidth(ESC, TopSpeed)

        else:
            GPIO.output(In1, GPIO.HIGH)
            GPIO.output(In2, GPIO.LOW)
            pwm1.ChangeDutyCycle(10)

            GPIO.output(In3, GPIO.HIGH)
            GPIO.output(In4, GPIO.LOW)
            pwm2.ChangeDutyCycle(10)

            GPIO.output(In5, GPIO.HIGH)
            GPIO.output(In6, GPIO.LOW)
            pwm3.ChangeDutyCycle(10)

            GPIO.output(In7, GPIO.HIGH)
            GPIO.output(In8, GPIO.LOW)
            pwm4.ChangeDutyCycle(10)

            pi.set_servo_pulsewidth(ESC, 0)

    cv2.imshow('Capture', frame)
    k = cv2.waitKey(1)
    if k == 27:
        break

cap.release()
cv2.destroyAllWindows()
