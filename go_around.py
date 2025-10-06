from lib.robot import *
import time, machine


from lib.robot_consts import Button, Port, Sensor, Light

MOTOR_BASE_POWER = 85           # 0-100 %
LIGHT_SETPOINT = 21             # 
Kp, Ki, Kd = 2.5, 2.5, 0.15           # PID constants

detection_time_s = 2
turn_aggression = 1.8
turn_base_power = 30


def go_around():
    robot.motors[Port.M1].set_power(-turn_base_power)
    robot.motors[Port.M2].set_power(turn_base_power)
    time.sleep(0.55)
    
    start_time = time.time()
    
    while 1:
        light_intensity = robot.sensors.light[Port.S1].reflection()
        if time.time() - start_time > (detection_time_s * 2/3)  and light_intensity < 25:
            
            
            return
            
        else:
            robot.motors[Port.M1].set_power(turn_base_power * turn_aggression)
            robot.motors[Port.M2].set_power(turn_base_power)
        


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

robot.sensors.laser[Port.S2].distance()


# Regulator variables
error = 0
last_error, integral, derivative = 0, 0, 0
output, motor_pwr = 0, 0


dt = 0.01
dist = 0



time.sleep_ms(2000)

while True:
    # Read light intensity
    light_intensity = robot.sensors.light[Port.S1].reflection()
    dist = robot.sensors.laser[Port.S2].distance()
    
    if dist < 100:
        go_around()
        last_error = 0
        integral = 0

    
    error = LIGHT_SETPOINT - light_intensity
    integral = integral +  error * dt

    derivative = (error - last_error) / dt
    output = Kp * error + Ki * integral + Kd * derivative
    last_error = error
    


    # Set motor power
    robot.motors[Port.M1].set_power(MOTOR_BASE_POWER + output)
    robot.motors[Port.M2].set_power(MOTOR_BASE_POWER - output)

    time.sleep_ms(int(dt * 1000))
    print("ss")
    # Exit program if left cube button is pressed
    buttons = robot.buttons.pressed()
    if buttons[Button.LEFT]:
        break
    
# Reset back to menu
if independent_run:
    machine.reset()
    
    
    
