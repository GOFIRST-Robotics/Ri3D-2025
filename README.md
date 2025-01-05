# Ri3D-2025
Official code from the UMN Robotics team for the 2025 Ri3D/FRC season!
 
 Our robot subsystems include:
 1) **Drivetrain:**  We chose to use a mechanim drive this year, and our available drivetrain commands include driving a specific distance, turning to a specific angle using a NavX gyroscope, and autonomously aiming at an Apriltag target and driving within range of it.
 2) **Intake:** The intake subsystem is responsible for collecting game elements from the floor. We chose to go with an intake powered by two NEO 550 motors, as it proved to be effective.
 3) **Climber:** Our climber is a modified (cut shorter) climber in the box from Andymark, powered by a NEO motor that is heavily geared down.
 4) **Vision:** We are running a Photonvision pipeline on a Raspberry Pi 3 B+ to detect Apriltags, and we can track and follow either the nearest Apriltag or an Apriltag with a specific ID.
 5) **LED Subsystem:** We wrote code for controlling RGB LED strips using a REV Blinkin LED driver to add some extra bling to our robot!
 6) **Power Subsystem:** This subsystem is for reading data from the REV PDH, such as the current draw of specific channels.
 7) **Coral Elevator:** This subsystem controls three motors. One is responsisble for moving the Elevator up and down and the other two control both the angle of the End Affector and the Finger for grabbing and dropping the coral.
 
How to get set up for FRC programming:
1) Install the latest 2025 release of WPILib and the latest 2025 release of the NI FRC Game tools.
2) Use this open-source Ri3D repository as a template for your code if you'd like! :)

Our project is created using the "Timed Robot" template/style and our code is written in Java.
