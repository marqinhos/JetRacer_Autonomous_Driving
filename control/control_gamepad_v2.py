
import pygame
from adafruit_servokit import ServoKit


i2c_address = 0x40
steering_channel = 0
throttle_channel = 1
kit = ServoKit(channels=16, address=i2c_address)
steering_motor = kit.continuous_servo[steering_channel]
throttle_motor = kit.continuous_servo[throttle_channel]

pygame.init()

done = False

controller = pygame.joystick.Joystick(0)	
controller.init()	
factor = 1
try:
	print("Detected joystick", controller.get_name())
	print("SCAN")
	while not done:

		events = pygame.event.get()
		for event in events:	
			if controller.get_button(1):
				break
			if controller.get_button(6): 
				factor -= 0.25
				factor = .1 if factor < 0 else factor
			if controller.get_button(7): 
				factor += 0.25 
				factor = 1 if factor > 1 else factor
			if controller.get_axis(0):
				value_turn = round(controller.get_axis(0), 2)*-1
			if controller.get_axis(3):
				value_vel = round(controller.get_axis(3), 2)*factor*-1
				
			throttle_motor.throttle = value_vel if abs(value_vel) <= 1 else 1 if value_vel > 1 else -1
			steering_motor.throttle = value_turn if abs(value_turn) <= 1 else 1 if value_turn > 1 else -1
			
except KeyboardInterrupt:
	print("EXITING NOW")
	controller.quit()  
	
"""

r = 2
t = 5
b = 1
x = 3
POSIBLE MEJORA PARA QUE CUANDO PULSAS NO SE QUEDE HASTA EL FONDO
if event.type == pygame.JOYBUTTONDOWN:
	
	if j.get_button(4):
		print("B")
		b = 2
	elif j.get_button(0):
		print("X")
		x = 5
elif event.type == pygame.JOYBUTTONUP:
	if b == r :
		print("b2")
		b = 1
	if x == t:
		print("x2")
		x = 3



"""
    
    
    
    
    
