import threading
import RPi.GPIO as GPIO #Servo
import cv2
import mlx90614
from machine import I2C, Pin
import pytesseract as tess
import numpy as np
import imutils
from time import sleep
from gpiozero import LED

hVictimDirection = ""
rescueDirection = ""
stopSign = False

#rotates the camera 
def servoStart(duty):
    servoPin = 17

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(servoPin,GPIO.OUT)
    servo = GPIO.PWM(servoPin,50)
    servo.start(0)

    servo.ChangeDutyCycle(duty)
    servo.stop()
    GPIO.cleanup()

#Defining rescueServo that will release rescue aid kits and turn according to rescueDirection
def rescueServo(rescueDuty):
    rServoPin = 16

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(rServoPin,GPIO.OUT)
    rescueServo = GPIO.PWM(rServoPin,50)
    rescueServo.start(0)

    rescueServo.ChangeDutyCycle(rescueDuty)
    servo.stop()
    GPIO.cleanup()

#Defining the function snap that will capture an img and save it for detecting colors n text
def snap():
    cam = cv2.VideoCapture(0)
    retval, frame = cam.read()
    cv2.imwrite('img.png', frame)

#Defining function blink that will be executed when any vitim is detected
def blink():
    ledPin = 18
    led = LED(ledPin)

    led.on()
    time.sleep(5)
    led.off()

#Defining rescueRelease function that takes direction and quantity as parameters
def rescueRelease(rescueDirection, quantity):
    i = 0

    while i < quantity:
        i += 1
        if rescueDirection == "R":
            rescueServo(2)
            rescueServo(7)
        else:
            rescueServo(12)
            rescueServo(7)

#defining hCheck function that will keep measuring temp while sending feedback to victimCheck
def heatCheck():
    global hVictimDirection

    i2c = I2C(scl=Pin(5), sda=Pin(4))
    sensor = mlx90614.MLX90614(i2c)

    #Initializing variables where the tempratures are recorded
    amb_temp = 0
    obj_temp = 0

    #As long as the stop sign isn't true, if the conditions are not satisfied measure temp:
    while (not stopSign):
        if (obj_temp > 28 and obj_temp < 40 and abs(abj_temp - amb_temp) >= 10):
            hVictimDirection = rescueDirection
        else:
            amb_temp = sensor.read_ambient_temp()
            obj_temp = sensor.read_object_temp()
        time.sleep(0.02)

#Defining Dilate function (makes letters thinner (BINARY)), using 3 iterations (2 was good, but 3 followed by 1 erode was best)
def dilate(image):
    kernel = np.ones((5, 5), np.uint8)
    return cv2.dilate(image, kernel, iterations=3)

#Defining Erode function (makes letter bolder (BINARY)), using 1 iteration (Found through testing, 2 was too much)
def erode(image):
    kernel = np.ones((5, 5), np.uint8)
    return cv2.erode(image, kernel, iterations=1)

#definig the function which will read the image and detect text and colors
def cvDetect(rescueDirection):
    #Reading the image in gray for preprocessing
    img = cv2.imread(r"/home/pi/Desktop/img.png", 0)

    blur = cv2.GaussianBlur(img, (5, 5), 0)
    s, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

    #Erode function on inverted image, blur for sharp edges
    thresh = dilate(thresh)
    thresh = erode(thresh)

    #Tesseract OCR on preprocessed image
    text = tess.image_to_string(thresh, config = "--psm 6")
    text = text.strip()

    #Visual Logic
    if "H" in text:
        rescueRelease(rescueDirection, 3)
        blink()
    if "S" in text or "s" in text or "5" in text:
        rescueRelease(rescueDirection, 2)
        blink()
    if "U" in text or "u" in text:
        blink()

    #Reading the image in color for color
    img = cv2.imread(r"/home/pi/Desktop/img.png", 1)

    #Detect color
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_red = np.array([0,150,50])
    upper_red = np.array([10,255,255])

    lower_green = np.array([65,60,60])
    upper_green = np.array([80,255,255])

    lower_yellow = np.array([25,150,50])
    upper_yellow = np.array([35,255,255])

    RedMask = cv2.inRange(img_hsv, lower_red, upper_red)
    GreenMask = cv2.inRange(img_hsv, lower_green, upper_green)
    YellowMask = cv2.inRange(img_hsv, lower_yellow, upper_yellow)

    RedCnts,Rh = cv2.findContours(RedMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    GreenCnts,Gh = cv2.findContours(GreenMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    YellowCnts,Yh = cv2.findContours(YellowMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #Initializing r,g,y variables for keeping track of areas
    r = 0
    g = 0
    y = 0

    cVictim = ""

    for c in RedCnts:
        RedArea = cv2.contourArea(c)
        if RedArea > 5:
            r += 1
    if r > 0:
        cVictim = "red"

    for c in GreenCnts:
        GreenArea = cv2.contourArea(c)
        if GreenArea > 5:
            g += 1
    if g > 0:
        cVictim = "green"

    for c in YellowCnts:
        YellowArea = cv2.contourArea(c)
        if YellowArea > 5:
            y += 1
    if y > 0:
        cVictim = "yellow"

    #Color logic
    if cVictim == "red":
        rescueRelease(rescueDirection, 1)
        blink()
    elif cVictim == "yellow":
        rescueRelease(rescueDirection, 1)
        blink()
    elif cVictim == "green":
        blink()

#Definig the color and visual check sequential function that will run async with hCheck
def victimCheck(victimDirection):
    global rescueDirection
    global stopSign

    #Starting the cvChecking
    if "R" in victimDirection:
        rescueDirection = "R"
        servoStart(2)
        snap()
        cvDetect(rescueDirection)
    if "L" in victimDirection:
        rescueDirection = "L"
        servoStart(12)
        snap()
        cvDetect(rescueDirection)

    stopSign = True

def mainCheck(victimDirection):
    t1 = threading.Thread(target=victimCheck,args=[victimDirection])
    t2 = threading.Thread(target=heatCheck)

    t1.start()
    t2.start()

    t1.join()
    t2.join()

    #reacting if the heat checking found any VICTIMS
    if hVictimDirection == "R":
        rescueRelease("R", 1)
        blink()
    if hVictimDirection == "L":
        rescueRelease("L", 1)
        blink()


if __name__ == "__main__":
    mainCheck("RL")
