import subprocess
import cv2
import numpy as np
import cv2
from ultralytics import YOLO

from gpiozero import Servo, AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep

factory = PiGPIOFactory()


# MG995 Servos
servo1 = Servo(17, pin_factory=factory, min_pulse_width=500/1000000, max_pulse_width=2400/1000000)
servo2 = Servo(18, pin_factory=factory, min_pulse_width=500/1000000, max_pulse_width=2400/1000000)
servo3 = Servo(27, pin_factory=factory, min_pulse_width=500/1000000, max_pulse_width=2400/1000000)



# MG90S Servos
servo4 = AngularServo(22, min_angle=-100, max_angle=100, pin_factory=factory)
servo5 = AngularServo(23, min_angle=-100, max_angle=100, pin_factory=factory)
servo6 = AngularServo(24, min_angle=-100, max_angle=100, pin_factory=factory)

#function for controlling MG995 motors...
def rotate_MG995(servo,start_value,end_value,step,delay):
    if start_value > end_value :
        # going in negative direction...
        for i in range(start_value,end_value,-step):
            servo.value = i/180
            sleep(delay)
        sleep(1)
    else:
        # going in positive direction...
        for i in range(start_value,end_value,step):
            servo.value = i/180
            sleep(delay)
        sleep(1)

#function for controlling MG90S motors...   
def rotate_MG90S(servo,start_angle,end_angle,step,delay):
    if start_angle > end_angle :
        # going in negative direction...
        for angle in range(start_angle,end_angle,-step):
            servo.angle = angle
            sleep(delay)
        sleep(1)
    else:
        # going in positive direction...
        for angle in range(start_angle,end_angle,step):
            servo.angle = angle
            sleep(delay)
        sleep(1)
    



# pick up the object and place it according the type of object
def robotic_hand(end_angle):
    #The end angle is for only the base motor
    
    #move motor 6 open claw
    rotate_MG90S(servo6,0,100,1,0.02)

    #move motor 3 go down
    rotate_MG995(servo3,0,-100,1,0.01)

    #move motor 6 hold object
    rotate_MG90S(servo6,99,0,1,0.02)

    #move motor 3 go up
    rotate_MG995(servo3,-99,0,1,0.01)
    
    #move motor 2 backward
    rotate_MG995(servo2,0,-90,1,0.04)

    # motor 1 Servos rotate right
    rotate_MG995(servo1,0,end_angle,1,0.05)
    
    #move motor 2 frontward
    rotate_MG995(servo2,-90,0,1,0.05)


    #move motor 6 to release
    rotate_MG90S(servo6,0,100,1,0.02)


    # motor 1 Servos rotete left
    rotate_MG995(servo1,end_angle,0,1,0.05)

    #move motor 6 close claw
    rotate_MG90S(servo6,100,0,1,0.02)





#capture the image from raspberry pi camera
def capture_image(output_dir, image_name):
    # Construct the command with the output directory and image name
    command = ['libcamera-still', '-o', f'{output_dir}/{image_name}']

    # Execute the command
    try:
        subprocess.run(command, check=True)
        print(f"Image captured and saved as '{output_dir}/{image_name}'")
    except subprocess.CalledProcessError as e:
        print(f"Error capturing image: {e}")








#for continuously run the process
while True:
    #capture an image
    output_directory = 'images'
    image_filename = 'image1.jpg'
    capture_image(output_directory, image_filename)

    # read the blank image incolour format , then resize it , then make it to gray scale for difference calculation
    empty_img = cv2.imread('/roboticArm/images/standard.jpg',1) #change the path if needed..
    empty_img = cv2.resize(empty_img,(700,700))
    empty_img1 = cv2.cvtColor(empty_img,cv2.COLOR_BGR2GRAY)

    # read the new image incolour format , then resize it , then make it to gray scale for difference calculation
    new_img = cv2.imread('roboticArm/images/image1.jpg',1) 
    new_img = cv2.resize(new_img,(700,700))
    new_img1 = cv2.cvtColor(new_img,cv2.COLOR_BGR2GRAY)

    #find pixel wise difference to identy , wheather any object is present or not
    diff = np.sum(cv2.absdiff(empty_img1,new_img1))
    print(diff)

    #if difference is above the thresold then object is present
    if(diff > 4000000):
        print('Object is present')
        print("Classifying the object...")
        model = YOLO('best.pt')
        result = model(new_img)
        
        for detection in result[0].boxes:
            class_id = int(detection.cls)
            #[Cotton,Gloves,bottle,mask,medicine,metal,syringe]
            # 0        1      2      3     4      5      6
            if(class_id == 0 or class_id == 1 or class_id == 3):
                robotic_hand(60)
            elif (class_id == 2 or class_id == 4 or class_id == 6):
                robotic_hand(180)
            else:
                robotic_hand(120)
        
            break
            
    else:
    	print('Object is not present')
sleep(3)
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
