from lib.robot import *
import time, machine
from lib.robot_consts import Sensor


from lib.robot_consts import Button, Port, Sensor, Light

MOTOR_BASE_POWER = 85           # 0-100 %




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

robot.init_sensor(sensor_type=Sensor.OC_COLOR, port=Port.S1)
robot.init_sensor(sensor_type=Sensor.OC_LASER, port=Port.S2)
#robot.init_sensor(sensor_type=Sensor.NXT_ULTRASONIC)


# Regulator variables

time.sleep_ms(5000)
motor_power = 0
dt = 0.01

phase = 1

while True:
    # Read light intensity
    light_intensity = robot.sensors.light[Port.S1].reflection()
    dist = robot.sensors.laser[Port.S2].distance()
    
    #scan
    robot.motors[Port.M1].set_power(-50 * phase)
    robot.motors[Port.M2].set_power(50 * phase)
    
    if dist < 1000:
        # thrust attack
        robot.motors[Port.M1].set_power(100)
        robot.motors[Port.M2].set_power(100)
        
        phase = -1
    else:
        phase = 1
        
        
    if light_intensity < 30 and False:
        robot.motors[Port.M1].set_power(-MOTOR_BASE_POWER)
        robot.motors[Port.M2].set_power(-MOTOR_BASE_POWER)
        time.sleep(1)

    
        

    time.sleep_ms(int(dt * 1000))
    print(dist, phase)
    # Exit program if left cube button is pressed
    buttons = robot.buttons.pressed()
    if buttons[Button.LEFT]:
        break
    
# Reset back to menu
if independent_run:
    machine.reset()
    
    
    

