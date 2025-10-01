from lib.robot import *
import time, machine


from lib.robot_consts import Button, Port, Sensor, Light

# TODO Change these constants
dt = 0.01      # Regulation period 0.010 s <=> 100 Hz
MOTOR_BASE_POWER = 50           # 0-100 %
LIGHT_SETPOINT = 21             # 
Kp, Ki, Kd = 5, 0, 0            # PID constants


# If the program is run from the menu, access global robot variable
# If it is run independently, robot is initialized
independent_run = False
global robot
try :
    robot
except :
    robot = Robot()
    independent_run = True
    

robot.init_motor(Port.M1)   #left motor
robot.init_motor(Port.M2)	#right motor

# Initialize Open-Cube RGB sensor on port S1
robot.init_sensor(sensor_type=Sensor.OC_COLOR, port=Port.S1)
# Initialize Open-Cube Touch Sensor on port S2
robot.init_sensor(sensor_type=Sensor.NXT_TOUCH,port=Port.S2)
# Regulator variables
error = 0
last_error, integral, derivative = 0, 0, 0
output, motor_pwr = 0, 0


#while not robot.sensors.touch[Port.S2]:
#    time.sleep(0.1)
time.sleep_ms(2000)

    
while True:
    light_intensity = robot.sensors.light[Port.S1].reflection() 
    
    error = LIGHT_SETPOINT - light_intensity
    
    integral = integral +  error * dt
    derivative = (error - last_error) / dt
    
    output = Kp * error + Ki * integral + Kd * derivative
    
    last_error = error
        

    # output /= 2.0 # divide the output between 2 motors
    motor_power = MOTOR_BASE_POWER# - abs(output)
    robot.motors[Port.M1].set_power(MOTOR_BASE_POWER + output)
    robot.motors[Port.M2].set_power(MOTOR_BASE_POWER - output)


    time.sleep_ms(dt * 1000)
    buttons = robot.buttons.pressed()
    if buttons[Button.LEFT]:
        break


# Reset back to menu
if independent_run:
    machine.reset()
    
    
    
    
    
    
