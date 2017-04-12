import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BOARD)
GPIO.setup(13, GPIO.OUT)
GPIO.output(13, 0)  #Set to low

def kick():
    GPIO.output(13, 1)  #Kick
    sleep(0.1)  #Wait a bit
    GPIO.output(13, 0)  #Retract
