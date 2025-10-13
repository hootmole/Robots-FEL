from lib.robot import *
import time, machine
from lib.robot_consts import Sensor


from lib.robot_consts import Button, Port, Sensor, Light

MOTOR_BASE_POWER = 85           # 0-100 %
LIGHT_SETPOINT = 21             # 
Kp, Ki, Kd = 2.6, 2.5, 0.15           # PID constants

detection_time_s = 2 * 2/3
turn_aggression = 1.8
turn_base_power = 30


def go_around(phase):
    if phase:
        robot.motors[Port.M1].set_power(-30)
        robot.motors[Port.M2].set_power(30)
    else:
        robot.motors[Port.M1].set_power(30)
        robot.motors[Port.M2].set_power(-30)
    
    time.sleep(0.55)
    
    start_time = time.time()
    
    while 1:
        light_intensity = robot.sensors.light[Port.S1].reflection()
        if time.time() - start_time > (detection_time_s) and light_intensity < 25:
            if phase:
                robot.motors[Port.M1].set_power(-15)
                robot.motors[Port.M2].set_power(15)
                time.sleep_ms(500)
                
            else:
                robot.motors[Port.M1].set_power(15)
                robot.motors[Port.M2].set_power(-15)
                time.sleep_ms(700)
                
            return
            
        else:
            if phase:
                robot.motors[Port.M1].set_power(turn_base_power * turn_aggression)
                robot.motors[Port.M2].set_power(turn_base_power)
            else:
                robot.motors[Port.M1].set_power(turn_base_power)
                robot.motors[Port.M2].set_power(turn_base_power * turn_aggression)
        


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
#robot.init_sensor(sensor_type=Sensor.OC_LASER, port=Port.S2)
robot.init_sensor(sensor_type=Sensor.NXT_ULTRASONIC)



# Regulator variables
error = 0
last_error, integral, derivative = 0, 0, 0
output, motor_pwr = 0, 0


dt = 0.01
dist = 0
stopping_dist = 100

phase = 1

time.sleep_ms(2000)
motor_power = 0

while True:
    # Read light intensity
    light_intensity = robot.sensors.light[Port.S1].reflection()
    #dist = robot.sensors.laser[Port.S2].distance()
    dist = robot.sensors.ultra_nxt.distance() * 10
    
    if (dist < 5 * stopping_dist) and not ((dist - stopping_dist) < 10):
        motor_power = MOTOR_BASE_POWER * (dist - stopping_dist) / (4 * stopping_dist)
        
    else: motor_power = MOTOR_BASE_POWER
    
    if (dist - stopping_dist) < 10:
        go_around(phase)
        last_error = 0
        integral = 0
        phase = (phase + 1) % 2

    
    error = LIGHT_SETPOINT - light_intensity
    integral = integral +  error * dt

    derivative = (error - last_error) / dt
    output = Kp * error + Ki * integral + Kd * derivative
    last_error = error
        

    # Set motor power
    robot.motors[Port.M1].set_power(motor_power + output)
    robot.motors[Port.M2].set_power(motor_power - output)

    time.sleep_ms(int(dt * 1000))
    print(dist)
    # Exit program if left cube button is pressed
    buttons = robot.buttons.pressed()
    if buttons[Button.LEFT]:
        break
    
# Reset back to menu
if independent_run:
    machine.reset()
    
    
    
