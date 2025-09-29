from lib.robot import *
import time, machine


from lib.robot_consts import Button, Port, Sensor, Light

# TODO Change these constants
REGULATION_PERIOD_MS = 10       # Regulation period 0.010 s <=> 100 Hz
MOTOR_BASE_POWER = 90           # 0-100 %
LIGHT_SETPOINT = 21             # 
Kp, Ki, Kd = 0.4, 10, 0.1            # PID constants


# If the program is run from the menu, access global robot variable
# If it is run independently, robot is initialized
independent_run = False
global robot
try :
    robot
except :
    robot = Robot()
    independent_run = True
    

# Initialize motors on ports M2 and M3
robot.init_motor(Port.M1)   #left motor
robot.init_motor(Port.M2)	#right motor

# Initialize Open-Cube RGB sensor on port S1
robot.init_sensor(sensor_type=Sensor.OC_COLOR, port=Port.S1)
# Initialize Open-Cube Touch Sensor on port S2
robot.init sensor(sensor type=Sensor.NXT TOUCH,port=Port.S2)
# Regulator variables
error = 0
last_error, integral, derivative = 0, 0, 0
output, motor_pwr = 0, 0

#output_min = -100
#output_max = 100
#integral_min = -10
#integral_max = 80
dt = 0.01


#while not robot.sensors.touch[Port.S2].pressed():
#    time.sleep_ms(300)


#while not robot.sensors.touch[Port]:
#    time.sleep(0.1)
time.sleep_ms(2000)

    
# Follower regulation loop
while True:
    # Read light intensity
    light_intensity = robot.sensors.light[Port.S1].reflection() 

    # TODO - PID regulator
    # 1. calculate error value e from light setpoint and measured ligth intensity
    # e =

    # 2. calculate motor_pwr using error variables (e, e_sum, e_prev) and PID constants (Kp, Ki, Kd)
    # motor_pwr = 

    # 3. save previous error for the derivative part
    # e_prev = 

    # 4. Save sum of errors for the integral part
    # e_sum =
    
    error = LIGHT_SETPOINT - light_intensity
    integral = integral +  error * dt
    #if(integral > integral_max):integral = integral_max
    #if(integral < integral_min):integral = integral_min
    derivative = (error - last_error) / dt
    output = Kp * error + Ki * integral + Kd * derivative
    last_error = error
    
    #if (output < output_min): output = output_min
    #if (output > output_max): output = output_max
    
    


    # Set motor power
    robot.motors[Port.M1].set_power(MOTOR_BASE_POWER + output)
    robot.motors[Port.M2].set_power(MOTOR_BASE_POWER - output)

    # Do nothing
    time.sleep_ms(REGULATION_PERIOD_MS)
    print("ss")
    # Exit program if left cube button is pressed
    buttons = robot.buttons.pressed()
    if buttons[Button.LEFT]:
        break
    
# Reset back to menu
if independent_run:
    machine.reset()
