# track_follower
Sensor Initialization:

The QTRSensorsAnalog object is created to manage the line tracking sensors.
The setup() function initializes the motor control pins and calibrates the sensors.
PID Control:

The loop() function reads the position of the line using the sensors.
It calculates the error between the desired set point (center of the line) and the actual position.
The PID algorithm computes the correction needed to align the robot with the line.
The motor speeds are adjusted based on the PID output to correct the robot’s path.
Motor Control:

The setMotorSpeed() function sets the speed and direction of the motors based on the calculated PID output.
