import math
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import imutils
import RPi.GPIO as gpio
import pigpio
import serial
from matplotlib import pyplot as plt
import smtplib
from datetime import datetime
from smtplib import SMTP
from smtplib import SMTPException
import email
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage


################### EMAIL #####################

def sendEmail(picture_time,status):
    # pic_time = "datetime.now().strftime('%Y%m%d%H%M%S')"
    pic_time = picture_time
    # Email information
    smtpUser = 'r8453571549@gmail.com'
    smtpPass = 'alaska@100'
    # Destination email information
    # toAdd = 'ENPM809TS19@gmail.com'
    toAdd = ['ENPM809TS19@gmail.com', 'skotasai@umd.edu', 'rpatil10@umd.edu']
    # toAdd = ['7rebellion@gmail.com']
    # carbon = 'skotasai@umd.edu'
    # they = '7rebellion@gmail.com'
    fromAdd = smtpUser
    subject = 'rpatil10@umd.edu Image recorded at '+ status + pic_time
    msg = MIMEMultipart()
    msg['Subject'] = subject
    msg['From'] = fromAdd
    msg['To'] = ",".join(toAdd)
    # msg['Cc'] = carbon
    # msg['Bcc'] = they
    msg.preamble = "rpatil10@umd.edu Image recorded at "+ status + pic_time
    # Email text
    body = MIMEText("rpatil10@umd.edu Image recorded at "+ status + pic_time)
    msg.attach(body)
    #Attach image
    fp = open(pic_time + '.jpg','rb')
    img = MIMEImage(fp.read())
    fp.close()
    msg.attach(img)
    # Send email
    s = smtplib.SMTP('smtp.gmail.com', 587)
    s.ehlo()
    s.starttls()
    s.ehlo()
    s.login(smtpUser, smtpPass)
    s.sendmail(fromAdd, toAdd, msg.as_string())
    s.quit()
    print("Email delivered!")
###############################################

################ MASTER INIT ###################
ser = serial.Serial('/dev/ttyUSB0', 9600)
def init():
    gpio.setmode(gpio.BOARD)
    gpio.setup(31, gpio.OUT) #IN1
    gpio.setup(33, gpio.OUT) #IN2
    gpio.setup(35, gpio.OUT) #IN3
    gpio.setup(37, gpio.OUT) #IN4
    gpio.setup(36, 0) # gripper servo zero
    gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP) # BR
    gpio.setup(7, gpio.IN, pull_up_down = gpio.PUD_UP) # FL

def gameover():
    gpio.output(31, False)
    gpio.output(33, False)
    gpio.output(35, False)
    gpio.output(37, False)
    gpio.cleanup()

def stop_cruise():
    global pwm31, pwm33, pwm35, pwm37
    gpio.output(31, False)
    gpio.output(33, False)
    gpio.output(35, False)
    gpio.output(37, False)
    pwm31.stop()
    pwm33.stop()
    pwm35.stop()
    pwm37.stop()

init()
pwm31 = gpio.PWM(31, 50) # LEFT
pwm37 = gpio.PWM(37, 50) # RIGHT
pwm33 = gpio.PWM(33, 50) # LEFT
pwm35 = gpio.PWM(35, 50) # RIGHT
##################################################

###############  GRIPPER INIT #############
pwm = pigpio.pi() 
pwm.set_mode(16, pigpio.OUTPUT)
pwm.set_PWM_frequency(16, 50)
pwm.set_servo_pulsewidth(16, 1750)

def my_range_increase(start, end, step):
    while start <= end:
        yield start
        start += step
def my_range_decrease(start, end, step):
    while start >= end:
        yield start
        start -= step

def close_gripper():
    pwm.set_servo_pulsewidth(16, 1750)
    for i in my_range_decrease(1750, 900, 10):
        time.sleep(0.05)
        pwm.set_servo_pulsewidth(16, i)
    
def open_gripper():
    pwm.set_servo_pulsewidth(16, 1225)
    for i in my_range_increase(900, 1750, 10):
        time.sleep(0.05)
        pwm.set_servo_pulsewidth(16, i)
###########################################

################## MOVEMENT #################
def moveForward(dist):
    global pwm31, pwm33, pwm35, pwm37
    counterBR = np.uint64(0)
    counterFL = np.uint64(0)
    buttonBR = int(0)
    buttonFL = int(0)
    valBR = 38
    valFL = 35
    rerror = 0
    lerror = 0
    Kp_BR = 1
    Kp_FL = 0.1 # for pwm val = 40 and dist = 1.0
    pwm35.stop()
    pwm33.stop()
    pwm37.start(38)
    pwm31.start(35)
    num_ticks_FL = np.int0(np.floor(dist*98))
    num_ticks_BR = np.int0(np.ceil(dist*98))
    iter = 0
    while True:
        if(int(gpio.input(7)) != int(buttonFL)):
            buttonFL = int(gpio.input(7))
            counterFL += 1  
        if int(gpio.input(12)) != int(buttonBR):
            buttonBR = int(gpio.input(12))
            counterBR += 1  
        rerror = counterFL - counterBR
        lerror = counterBR - counterFL
        pwm_value_BR = valBR + int(rerror*Kp_BR)
        pwm_value_FL = valFL + int(lerror*Kp_FL)
        if(pwm_value_BR > 100):
            pwm_value_BR = 100
        elif pwm_value_BR < 20:
            pwm_value_BR = 20
        if(pwm_value_FL >= 100):
            pwm_value_FL = 100
        elif pwm_value_FL <= 20:
            pwm_value_FL = 20
        pwm37.ChangeDutyCycle(pwm_value_BR)
        pwm31.ChangeDutyCycle(pwm_value_FL) 
        if counterBR >= num_ticks_BR and counterFL >= num_ticks_FL:
            pwm31.stop()
            pwm37.stop()
            break

def moveReverse(dist):
    global pwm31, pwm33, pwm35, pwm37
    counterBR = np.uint64(0)
    counterFL = np.uint64(0)
    buttonBR = int(0)
    buttonFL = int(0)
    valBR = 38
    valFL = 35
    rerror = 0
    lerror = 0
    Kp_BR = 1
    Kp_FL = 0.1 # for pwm val = 40 and dist = 1.0
    pwm31.stop()
    pwm37.stop()
    pwm33.start(38)
    pwm35.start(35)
    num_ticks_FL = np.int0(np.floor(dist*98))
    num_ticks_BR = np.int0(np.ceil(dist*98))
    iter = 0
    while True:
        if(int(gpio.input(7)) != int(buttonFL)):
            buttonFL = int(gpio.input(7))
            counterFL += 1 
        if int(gpio.input(12)) != int(buttonBR):
            buttonBR = int(gpio.input(12))
            counterBR += 1 
        rerror = counterFL - counterBR
        lerror = counterBR - counterFL
        pwm_value_BR = valBR + int(rerror*Kp_BR)
        pwm_value_FL = valFL + int(lerror*Kp_FL)
        if(pwm_value_BR > 100):
            pwm_value_BR = 100
        elif pwm_value_BR < 20:
            pwm_value_BR = 20
        if(pwm_value_FL >= 100):
            pwm_value_FL = 100
        elif pwm_value_FL <= 20:
            pwm_value_FL = 20
        pwm35.ChangeDutyCycle(pwm_value_BR)
        pwm33.ChangeDutyCycle(pwm_value_FL)
        if counterBR >= num_ticks_BR and counterFL >= num_ticks_FL:
            pwm33.stop()
            pwm35.stop()
            break

def turn_right():
    global pwm31, pwm33, pwm35, pwm37
    pwm33.stop()
    pwm37.stop()
    pwm31.start(46)
    pwm35.start(46)

def turn_left():
    global pwm31, pwm33, pwm35, pwm37
    pwm31.stop()
    pwm35.stop()
    pwm33.start(46)
    pwm37.start(46)

def turn_right_adj():
    global pwm31, pwm33, pwm35, pwm37
    pwm33.stop()
    pwm37.stop()
    pwm31.start(41)
    pwm35.start(41)

def turn_left_adj():
    global pwm31, pwm33, pwm35, pwm37
    pwm31.stop()
    pwm35.stop()
    pwm33.start(41)
    pwm37.start(41)

def degree_to_encoder_ticks(degree):
    return int((20/(np.pi*0.065)) * (0.18*np.deg2rad(degree)))
#################################################################

################  ALIGNMENT FUNCTIONS #######################
def check_deviation_and_turn_right(x_diff):
    buttonBR = int(0)
    buttonFL = int(0)
    diff_in_degree = int(x_diff*0.061)
    encoder_ticks = degree_to_encoder_ticks(diff_in_degree)
    counterBR = np.uint64(0)
    counterFL = np.uint64(0)
    while(counterBR < encoder_ticks or counterFL < encoder_ticks):
        if int(gpio.input(12)) != int(buttonBR):
            buttonBR = int(gpio.input(12))
            counterBR += 1
        if int(gpio.input(7)) != int(buttonFL):
            ButtonFL = int(gpio.input(7))
            counterFL += 1
        turn_right_adj()
        
def check_deviation_and_turn_left(x_diff):
    buttonBR = int(0)
    buttonFL = int(0)
    diff_in_degree = int(x_diff*0.061)
    encoder_ticks = degree_to_encoder_ticks(diff_in_degree)
    counterBR = np.uint64(0)
    counterFL = np.uint64(0)
    while(counterBR < encoder_ticks or counterFL < encoder_ticks):
        if int(gpio.input(12)) != int(buttonBR):
            buttonBR = int(gpio.input(12))
            counterBR += 1
        if int(gpio.input(7)) != int(buttonFL):
            ButtonFL = int(gpio.input(7))
            counterFL += 1
        turn_left_adj()
################################################################

##############  IMU ############
def get_imu_reading():
    global ser
    ser.reset_input_buffer()
    while (ser.in_waiting == 0):
        continue
    line = ser.readline()
    line = line.rstrip().lstrip()
    line = str(line)
    line = line.strip("'")
    line = line.strip("b'")
    line = float(line)
    return line
################################

############# COLORS #############
redLower = (0,120,64)
redUpper = (17,255,255)
greenLower = (28,95,43)
greenUpper = (95,255,255)
blueLower = (80,116,30)
blueUpper = (127,255,255)
##################################

############# CAMERA SETTINGS #################
camera = PiCamera()
camera.resolution = (640,480)
camera.framerate = 36
rawCapture = PiRGBArray(camera, size=(640,480))
################################################

################ LOCALIZATION INIT ################
trajectory = []
previous_status = [int(0),int(0),int(0)]
current_status = [int(0),int(0),int(0)]
drop_area = [int(30),int(300),int(315)]
print("[X, Y, Direction]: "+str(current_status))
trajectory.append(current_status[:])
def update_robot_orientation(turn_direction):
    global current_status, previous_status
    current_status[2] = int(turn_direction)

def update_robot_location(dist, move_direction, g_angle):
    global current_status, previous_status
    curr_x = current_status[0]
    curr_y = current_status[1]
    curr_theta = current_status[2]
    if(move_direction=="forward"):
        if((g_angle>=0) and (g_angle<90)):
            x = dist*100* math.cos((90 - curr_theta)*np.pi/180)
            y = dist*100* math.sin((90 - curr_theta)*np.pi/180)
            current_status[0] = curr_x + abs(x)
            current_status[1] = curr_y + abs(y)
        elif((g_angle>=90) and (g_angle<180)):
            x = dist*100* math.cos((curr_theta - 90)*np.pi/180)
            y = dist*100* math.sin((curr_theta - 90)*np.pi/180)
            current_status[0] = curr_x + abs(x)
            current_status[1] = curr_y - abs(y)
        elif((g_angle>=180) and (g_angle<270)):
            x = dist*100* math.cos((270 - curr_theta)*np.pi/180)
            y = dist*100* math.sin((270 - curr_theta)*np.pi/180)
            current_status[0] = curr_x - abs(x)
            current_status[1] = curr_y - abs(y)
        elif((g_angle>=270) and (g_angle<360)):
            x = dist*100* math.cos((curr_theta - 270)*np.pi/180)
            y = dist*100* math.sin((curr_theta - 270)*np.pi/180)
            current_status[0] = curr_x - abs(x)
            current_status[1] = curr_y + abs(y)
    if(move_direction=="reverse"):
        if((g_angle>=0) and (g_angle<90)):
            x = dist*100* math.cos((90 - curr_theta)*np.pi/180)
            y = dist*100* math.sin((90 - curr_theta)*np.pi/180)
            current_status[0] = curr_x - abs(x)
            current_status[1] = curr_y - abs(y)
        elif((g_angle>=90) and (g_angle<180)):
            x = dist*100* math.cos((curr_theta - 90)*np.pi/180)
            y = dist*100* math.sin((curr_theta - 90)*np.pi/180)
            current_status[0] = curr_x - abs(x)
            current_status[1] = curr_y + abs(y)
        elif((g_angle>=180) and (g_angle<270)):
            x = dist*100* math.cos((270 - curr_theta)*np.pi/180)
            y = dist*100* math.sin((270 - curr_theta)*np.pi/180)
            current_status[0] = curr_x + abs(x)
            current_status[1] = curr_y + abs(y)
        elif((g_angle>=270) and (g_angle<360)):
            x = dist*100* math.cos((curr_theta - 270)*np.pi/180)
            y = dist*100* math.sin((curr_theta - 270)*np.pi/180)
            current_status[0] = curr_x + abs(x)
            current_status[1] = curr_y - abs(y)
    trajectory.append(current_status[:])
    
    

update_robot_orientation(get_imu_reading())
time.sleep(4)
update_robot_orientation(get_imu_reading())
####################################################

################### DROP LOCATION CALC #############
def calc_distance_and_angle_to_drop():
    global current_status, drop_area
    x1 = current_status[0]
    y1 = current_status[1]
    x2 = drop_area[0]
    y2 = drop_area[1]
    del_X = x2 - x1
    del_Y = y2 - y1
    m = (del_Y)/(del_X)
    theta = int(abs(math.atan(m)*180/np.pi))
    d = math.sqrt((abs(del_X)**2)+(abs(del_Y)**2))
    d = d/100
    return theta, d
####################################################

stop_cruise()

############## FLAGS #################
start_point_survey = True
turn_after_drop = False
is_object_at_gripper = False
is_object_picked_up = False
is_object_dropped_to_goal = False
retrival_go_straight = False
retrival_go_turn = False
retrival_go_align = False
mission_status = False
found_next_block = False
######################################
######################################
global_theta_to_goal = int(0)
global_distance_to_goal = int(0)
drop_later_theta = int(0)
pick_up_block_color = "red"
color_lower = (0,128,56)
color_upper = (17,255,255)
counter = 0
heading_for_sweep = "east"
######################################


############################################################


for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    #grab the current frame
    image = frame.array
    blank_image = np.zeros((480,640,3), np.uint8)
    blank_image[:,:] = (255,255,255)
    cover_top = np.zeros((480,640,3), np.uint8)
    cover_top[240:480,:] = (255,255,255)
    output = cv2.bitwise_and(image, cover_top, mask=None)
    hsv = cv2.cvtColor(output, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, color_lower, color_upper)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None
    image_center = int(640/2)
    cv2.line(image, (int(640*0.4),int(480/2)),(int(640*0.6),int(480/2)), (255,255,255), 2)
    cv2.line(image, (int(640/2),int(480*0.4)),(int(640/2),int(480*0.6)), (255,255,255), 2)
    if(cnts):
        c = max(cnts, key=cv2.contourArea)
        ((x,y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        try:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        except:
            cv2.imshow("frame", image)
            key = cv2.waitKey(1) & 0xFF
            rawCapture.truncate(0)
            if key == ord("q"):
                break
        if radius > 10:
            cv2.circle(image,(int(x), int(y)), int(radius), (0,255,255), 2)
            cv2.circle(image, center, 5, (0, 255, 255), -1)
        cv2.putText(image,str(center), (40,40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        if(start_point_survey):
            if(x>image_center):
                diff_in_pixels = x - image_center
                if(diff_in_pixels < 10):
                    start_point_survey = False
                    stop_cruise()
                    update_robot_orientation(get_imu_reading())
                else:
                    check_deviation_and_turn_right(diff_in_pixels)
            if(x<image_center):
                diff_in_pixels = image_center - x
                if(diff_in_pixels < 10):
                    start_point_survey = False
                    stop_cruise()
                    update_robot_orientation(get_imu_reading())
                else:
                    check_deviation_and_turn_left(diff_in_pixels)
        if((not is_object_picked_up) and (not start_point_survey)):
            if(radius > 145):
                stop_cruise()
                pwm.set_servo_pulsewidth(16, 900)
                update_robot_orientation(get_imu_reading())
                cap_time = datetime.now().strftime('%Y%m%d%H%M%S')
                cv2.imwrite(cap_time + '.jpg', image)
                sendEmail(cap_time,"Picked up")
                theta_to_goal, global_distance_to_goal = calc_distance_and_angle_to_drop()
                global_theta_to_goal = 270+theta_to_goal
                is_object_picked_up = True
            else:
                turn_after_drop = False
                found_next_block = False
                g_a = int(get_imu_reading())
                if(radius>65):
                    moveForward(0.05)
                else:
                    moveForward(0.1)
                stop_cruise()
                update_robot_location(0.1,"forward", g_a)
                if(x>image_center):
                    diff_in_pixels = x - image_center
                    if(diff_in_pixels < 48):
                        stop_cruise()
                        update_robot_orientation(get_imu_reading())
                    else:
                        check_deviation_and_turn_right(diff_in_pixels)
                if(x<image_center):
                    diff_in_pixels = image_center - x
                    if(diff_in_pixels < 48):
                        start_point_survey = False
                        stop_cruise()
                        update_robot_orientation(get_imu_reading())
                    else:
                        check_deviation_and_turn_left(diff_in_pixels)
        if((is_object_picked_up) and (not retrival_go_turn)):
            xyz = int(get_imu_reading())
            if(not(xyz >= int(global_theta_to_goal-10) and xyz <= int(global_theta_to_goal+10))):
                turn_left()
            else:
                stop_cruise()
                retrival_go_turn = True
                update_robot_orientation(get_imu_reading())
        if((is_object_picked_up) and (retrival_go_turn) and (not retrival_go_straight)):
            g_a = int(get_imu_reading())
            moveForward(global_distance_to_goal)
            stop_cruise()
            update_robot_location(global_distance_to_goal,"forward", g_a)
            retrival_go_straight = True
        if((is_object_picked_up) and (retrival_go_turn) and (retrival_go_straight) and (not is_object_dropped_to_goal)):
            pwm.set_servo_pulsewidth(16, 1750)
            g_a = int(get_imu_reading())
            moveReverse(0.2)
            stop_cruise()
            update_robot_location(0.1,"reverse", g_a)
            counter = counter + 1
            if(counter == 8):
                break
            drop_later_theta = 90
            if(counter==1 or counter==4 or counter==7):
                color_lower = greenLower
                color_upper = greenUpper
            if(counter==0 or counter==3 or counter==6):
                color_lower = redLower
                color_upper = redUpper
            if(counter==2 or counter==5 or counter==8):
                color_lower = blueLower
                color_upper = blueUpper
            is_object_dropped_to_goal = True
        if((is_object_picked_up) and (retrival_go_turn) and (retrival_go_straight) and (is_object_dropped_to_goal) and (not turn_after_drop)):
            abc = int(get_imu_reading())
            if(not(abc >= int(drop_later_theta-10) and abc <= int(drop_later_theta+10))):
                turn_right()
            else:
                stop_cruise()
                update_robot_orientation(get_imu_reading())
                is_object_picked_up = False
                retrival_go_align = False
                retrival_go_turn = False
                retrival_go_straight = False
                is_object_dropped_to_goal = False
                turn_after_drop = True
                start_point_survey = True
                
        cv2.imshow("frame", image)
    else:
        if(start_point_survey):
            turn_right()
        cv2.imshow("frame", image)
    key = cv2.waitKey(1) & 0xFF
    rawCapture.truncate(0)
    if key == ord("q"):
        break

print("[X, Y, Direction]: "+str(current_status))
pwm.set_servo_pulsewidth(16, 1750)
pwm.set_PWM_dutycycle(16, 0)
pwm.set_PWM_frequency(16, 0)
stop_cruise()
gameover()
trajectory_location_only = [l[:2] for l in trajectory]
trajectory_x_only = [l[0] for l in trajectory_location_only]
trajectory_y_only = [j[1] for j in trajectory_location_only]
plt.plot(trajectory_x_only,trajectory_y_only)
plt.xlim([-10, 330])
plt.ylim([-10, 330])
plt.show()
plt.savefig('traversal_plot.png')