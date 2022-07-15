import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

Ena,In1,In2 = 26,27,22    # left motor
Enb,In3,In4 = 4,23,24     # right motor

S1,S2,S3,S4,S5 = 5,6,13,12,25  # sensor pins

##MotorSetup
GPIO.setup(Ena,GPIO.OUT)
GPIO.setup(Enb,GPIO.OUT)
GPIO.setup(In1,GPIO.OUT)
GPIO.setup(In2,GPIO.OUT)
GPIO.setup(In3,GPIO.OUT)
GPIO.setup(In4,GPIO.OUT)

##PWMsetup
pwm1 = GPIO.PWM(Ena,200)
pwm2 = GPIO.PWM(Enb,200)
pwm1.start(100)
pwm2.start(100)

##SensorSetup
GPIO.setup(S1,GPIO.IN)
GPIO.setup(S2,GPIO.IN)
GPIO.setup(S3,GPIO.IN)
GPIO.setup(S4,GPIO.IN)
GPIO.setup(S5,GPIO.IN)

##constants
error = 0
Kp,Ki,Kd = 6,0,1
P,I,D = 0,0,0
previous_error,previous_I = 0,0

## initializeSpeed
initial_motor_speed = 30


sensor = [0,0,0,0,0] # 0 denotes black line & 1 denotes white

def read_sensor_values():
    
    global error
    sensor[0] = GPIO.input(S1)
    sensor[1] = GPIO.input(S2)
    sensor[2] = GPIO.input(S3)
    sensor[3] = GPIO.input(S4)
    sensor[4] = GPIO.input(S5)
    
    
    if ((sensor[0] == 1) and (sensor[1] == 1) and (sensor[2] == 1) and (sensor[3] == 1) and (sensor[4] == 0)):
        #11110
        error = 4
    
    if ((sensor[0] == 1) and (sensor[1] == 1) and (sensor[2] == 1) and (sensor[3] == 0) and (sensor[4] == 0)):
        #11100
        error = 3
        
    if ((sensor[0] == 1) and (sensor[1] == 1) and (sensor[2] == 1) and (sensor[3] == 0) and (sensor[4] == 1)):
        #11101
        error = 2
    
    if ((sensor[0] == 1) and (sensor[1] == 1) and (sensor[2] == 0) and (sensor[3] == 0) and (sensor[4] == 1)):
        #11001
        error = 1
        
    if ((sensor[0] == 1) and (sensor[1] == 1) and (sensor[2] == 0) and (sensor[3] == 1) and (sensor[4] == 1)):
        #11011
        error = 0
        
    if ((sensor[0] == 1) and (sensor[1] == 0) and (sensor[2] == 0) and (sensor[3] == 1) and (sensor[4] == 1)):
        #10011
        error = -1
        
    if ((sensor[0] == 1) and (sensor[1] == 0) and (sensor[2] == 1) and (sensor[3] == 1) and (sensor[4] == 1)):
        #10111
        error = -2
        
    if ((sensor[0] == 0) and (sensor[1] == 0) and (sensor[2] == 1) and (sensor[3] == 1) and (sensor[4] == 1)):
        #00111
        error = -3
        
    if ((sensor[0] == 0) and (sensor[1] == 1) and (sensor[2] == 1) and (sensor[3] == 1) and (sensor[4] == 1)):
        #01111
        error = -4
        
    if ((sensor[0] == 1) and (sensor[1] == 1) and (sensor[2] == 1) and (sensor[3] == 1) and (sensor[4] == 1)):
        #11111
        if (error == -4):
            error = -5
        else:
            error = 5
            
    # sharp turns
    
    if ((sensor[0] == 0) and (sensor[1] == 0) and (sensor[2] == 0) and (sensor[3] == 1) and (sensor[4] == 1)):
        #00011
        error = 100
        print("*************************")
        
    if ((sensor[0] == 0) and (sensor[1] == 0) and (sensor[2] == 0) and (sensor[3] == 0) and (sensor[4] == 1)):
        #00011
        error = 100
        print("*************************")
            
    print(sensor)
    print(error)
    print("...........")
    
    
    return error

def constrain(val, min_val, max_val):

    if val < min_val: 
        val = min_val

    elif val > max_val: 
        val = max_val

    return val
    
    
def calculate_pid():
    global I
    global previous_error
    P = error
    I = I + previous_error
    D = error - previous_error
    
    PID_value = (Kp*P) + (Ki*I) + (Kd*D)
    previous_I = I
    previous_error = error
    return PID_value
    
def motor_control():
    left_motor_speed = initial_motor_speed-PID_value
    right_motor_speed = initial_motor_speed+PID_value
    
    left_motor_speed = constrain(left_motor_speed,0,100)
    right_motor_speed = constrain(right_motor_speed,0,100)
    
    pwm2.ChangeDutyCycle(left_motor_speed)
    pwm1.ChangeDutyCycle(right_motor_speed)
    forward()
    
def forward():
    GPIO.output(In1,GPIO.HIGH)
    GPIO.output(In2,GPIO.LOW)
    GPIO.output(In3,GPIO.LOW)
    GPIO.output(In4,GPIO.HIGH)
    
def reverse():
    GPIO.output(In1,GPIO.LOW)
    GPIO.output(In2,GPIO.HIGH)
    GPIO.output(In3,GPIO.HIGH)
    GPIO.output(In4,GPIO.LOW)
    
def right():
    GPIO.output(In1,GPIO.HIGH)
    GPIO.output(In2,GPIO.LOW)
    GPIO.output(In3,GPIO.LOW)
    GPIO.output(In4,GPIO.LOW)
    
def left():
    GPIO.output(In1,GPIO.LOW)
    GPIO.output(In2,GPIO.LOW)
    GPIO.output(In3,GPIO.LOW)
    GPIO.output(In4,GPIO.HIGH)
    
def sharp_right():
    GPIO.output(In1,GPIO.HIGH)
    GPIO.output(In2,GPIO.LOW)
    GPIO.output(In3,GPIO.HIGH)
    GPIO.output(In4,GPIO.LOW)
    
def sharp_left():
    GPIO.output(In1,GPIO.LOW)
    GPIO.output(In2,GPIO.HIGH)
    GPIO.output(In3,GPIO.LOW)
    GPIO.output(In4,GPIO.HIGH)
    
def stop():
    GPIO.output(In1,GPIO.LOW)
    GPIO.output(In2,GPIO.LOW)
    GPIO.output(In3,GPIO.LOW)
    GPIO.output(In4,GPIO.LOW)
    
    
    
      
while 1:
    read_sensor_values()
    
    if (error == 100):
        while True:
            read_sensor_values()
            pwm2.ChangeDutyCycle(25)
            pwm1.ChangeDutyCycle(25)
            sharp_left()
            time.sleep(0.25)
            if (error != 0):
                break
    
    else:
        PID_value=calculate_pid()
        motor_control()
