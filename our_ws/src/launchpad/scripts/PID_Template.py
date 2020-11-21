import numpy as np 



center_line = 5 
center_robot = 0 


DEFAULT_SPEED = 1
SCALE_FACTOR = 1 

#PID Parameters
Kp = 1;
Ki = 0;
Kd = 0;

#Calculate error  
x_error = center_line - center_robot 


running_error = 0 if (np.sign(x_error) != np.sign(prev_error) || (x_error <= 0.01 && x_error >= -0.01 ) ) else running_error + x_error
delta_error = error-prev_error 



#Calculate Motor Offest
motor_offset = x_error * Kp + Ki * running_error + Kd * delta_error
motor_offset_comp = 1 - abs(motor_offset)

right_speed = DEFAULT_SPEED
left_speed = DEFAULT_SPEED

#Set Motor Speed
if(motor_offset > 0 ):
	right_speed = DEFAULT_SPEED + motor_offset*SCALE_FACTOR
	left_speed = DEFAULT_SPEED - motor_offset*SCALE_FACTOR
elif(motor_offset < 0 ):
	left_speed = DEFAULT_SPEED + motor_offset*SCALE_FACTOR
	right_speed = DEFAULT_SPEED - motor_offset*SCALE_FACTOR


#Save Current Error 
prev_error = error 

