# Ri3D-2025
Official code from the UMN Robotics team for the 2025 Ri3D/FRC season!
 
Our robot subsystems include:
1) **Drivetrain:** We chose to use a mecanum drive this year for better maneuverability compared to tank drive, and our available drivetrain commands include driving a specific distance, turning to a specific angle using a NavX gyroscope, and autonomously aiming at an Apriltag target and driving within range of it.
2) **Elevator:** The elevator subsystem raises and lowers and has an end effector that is responsible for scoring the coral game elements.
3) **Intake:** The intake subsystem is responsible for collecting Algae game elements from the floor. We chose to go with an intake roller powered by a NEO 550 motor, and it is actuated in and out using another NEO 550 motor heavily geared down.
4) **Climber:** Our climber is a passive mechanism on the linear actuator of the elevator. We are still working on it!
5) **Vision:** We are running a Photonvision pipeline on an Orange Pi 5 to detect Apriltags, and we can track and follow either the nearest Apriltag or an Apriltag with a specific ID.
6) **LED Subsystem:** We wrote code for controlling RGB LED strips using a REV Blinkin LED driver to add some extra bling to our robot!
7) **Power Subsystem:** This subsystem is for reading data from the REV PDH, such as the current draw of specific channels.

How to get set up for FRC programming:
1) Install the latest 2025 release of WPILib and the latest 2025 release of the NI FRC Game tools.
2) Use this open-source Ri3D repository as a template for your code if you'd like! :)

Our project is created using the "Timed Robot" template/style and our code is written in Java.
