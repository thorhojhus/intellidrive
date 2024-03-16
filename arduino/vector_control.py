import pygame
import serial
import time

# Initialize Pygame
pygame.init()

# Initialize the joystick
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    print("No joystick detected.")
    quit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

# Initialize serial communication with Arduino
ser = serial.Serial('/dev/ttyUSB0', 115200)
time.sleep(2)  # Wait for Arduino to initialize

# Main loop
running = True
while running:
    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Get controller input
    axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]

    # send steering and throttle to arduino
    steering = int((axes[0] + 1) * 100) # Left analog
    throttle = int((axes[4] + 2) * 100) # Right trigger
    if axes[5] > - 0.98:
        throttle = 100 - int((axes[5] + 2) * 100)
    
    # send as binary
    ser.write(bytearray([steering, throttle]))

# Clean up
pygame.quit()
