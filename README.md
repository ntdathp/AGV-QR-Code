# Qr-Control

### This repo consists of firmware, software and hardward of my thesis about AGV QR code:
The thesis designs a solution using a QR code matrix set up to navigate and locate an AGV. The AGV will follow a trajectory defined by the user, with specified start and end points

### Hardware Design:
Develop a robot model with a downward-facing camera to observe QR codes on the ground. A Raspberry Pi is used to process images from the camera, communicate with the GUI, and control the robot's movement via UART communication with a custom-designed STM32 microcontroller circuit. This microcontroller circuit will control the PID velocity of two DC motors using a custom-designed H-bridge circuit.

### PID Controller and Trapizoidal Velocity Profile:
Using trapezoidal velocity information and a PID velocity controller to move the vehicle within the QR code matrix.

### GUI:
The GUI interface is programmed using Qt, communicating with the robot through the MQTT protocol. It sends the movement trajectory between two endpoints set by the user, notifies when the vehicle reaches the QR codes in the matrix, and adjusts the vehicle's speed.

#### Go to goal process: 
Frames from Camera             |  Image processing with Optical Flow algorithm
:-------------------------:|:-------------------------:

### Test environment 
The QR code matrix is arranged on the floor, and a camera positioned above looks down to observe during the movement process to evaluate the results
