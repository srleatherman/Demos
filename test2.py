#state when the order has not been started
value='0'
quad='0'

#on off switch
in_use=1

# inprots all necicary librarys
import numpy as np
import cv2 as cv
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import time
import board
import serial
from time import sleep
from picamera import PiCamera

#set up of the camara's conection to the pi
##
##camera = PiCamera(resolution=(1280, 720), framerate=30)
##camera.iso = 100
##sleep(2)
##camera.shutter_speed = camera.exposure_speed
##camera.exposure_mode = 'off'
##g = camera.awb_gains
##camera.awb_mode = 'off'
##camera.awb_gains = g
##camera.capture_sequence(['image%02d.jpg' % i for i in range(10)])


# sets up the lcd size
lcd_columns = 16
lcd_rows = 2

# Initialise I2C bus.
i2c = board.I2C()

# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

#ready the lcd
lcd.clear()
lcd.color = [100, 0, 100]


#Set address
#ser = serial.Serial('/dev/ttyACM0', 115200)

#Wait for connection to complete
#time.sleep(3)

def find_quadrant():
        #Camera initialization and capturing an image
        camera = PiCamera(resolution=(1920,1080), framerate=30)
        camera.capture('/home/pi/mkdir/screen.jpg')
        camera.close()
        #Reading image as cv object
        img = cv.imread('screen.jpg')
        #Converting from bgr to hsv
        img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        #Upper and lower limits for color detection
        red_lower = np.array([100,0,0])
        red_upper = np.array([115,200,255])
        #Isolating red
        mask = cv.inRange(img_hsv, red_lower, red_upper)
        #Applying filters and effects to sharpen mask
        imgmanip = cv.bitwise_and(img_hsv, img_hsv, mask = mask)
        imgmanip = cv.cvtColor(imgmanip, cv.COLOR_HSV2BGR)
        imgmanip = cv.cvtColor(imgmanip, cv.COLOR_BGR2GRAY)
        imgmanip = cv.GaussianBlur(imgmanip, (5,5), 0)
        imgmanip = cv.threshold(imgmanip, 60, 255, cv.THRESH_BINARY)[1]
        cv.imwrite('mask.jpg', imgmanip)
        #Detecting contours
        cnts = cv.findContours(imgmanip.copy(), cv.RETR_LIST,cv.CHAIN_APPROX_SIMPLE)[1]
        cX = 0
        cY = 0
        
        #Find the location of the marker
        for c in cnts:
            M = cv.moments(c)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
        lcd.clear()

        if cX == 0 and cY == 0:  #alerts to the marker not being found
            lcd.message = "No markers found"
            return '0'
        
        else:  # retuns the quad number and displays the location
            #print(screen.size)
            #print(cX," ",cY)
                angle=(60/2)*((cX-960)/(1920-960))
                rounded= round(angle, 2)
                send=str(rounded)
                lcd.message = send
                return send
#sets up a funtion that can read the arduino. used for debugging
def ReadfromArduino():
    while (ser.in_waiting > 0):
        try:
            line = ser.readline().decode('utf-8').rstrip()
            print("serial output : ", line)
        except:
            print("Communication Error")
            break


while in_use == 1:
        
        #uses the cammara to find the quadrint
        send=find_quadrant()
        if send != '0':
                in_use=0
        #wries the quad to the arduino
        #ser.write(send.encode())
        
##      #wait
        #time.sleep(2)
        #debuging return
        #ReadfromArduino()
##        time.sleep(2)

