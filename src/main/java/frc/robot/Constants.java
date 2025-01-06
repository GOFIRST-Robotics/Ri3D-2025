
// Author: UMN Robotics Ri3D
// Last Updated: December 2024

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) 
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Physical Robot Constants //
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(6); // Convert from inches to meters
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER; // Measured in meters
	public static final double TRACK_WIDTH = Units.inchesToMeters(21); // Distance between centers of right and left wheels on robot (in meters)
    public static final double WHEEL_BASE = Units.inchesToMeters(16.75); // Distance between centers of front and back wheels on robot (in meters)

    // Controller Input Axes //
    public static final int CONTROLLER_USB_PORT_ID = 0; // USB port that the controller is plugged in to
    public static final int RIGHT_VERTICAL_JOYSTICK_AXIS = 3;
    public static final int RIGHT_HORIZONTAL_JOYSTICK_AXIS = 2;
    public static final int LEFT_VERTICAL_JOYSTICK_AXIS = 1;
    public static final int LEFT_HORIZONTAL_JOYSTICK_AXIS = 0;
    public static final int X_BUTTON = 1;
    public static final int A_BUTTON = 2;
    public static final int B_BUTTON = 3;
    public static final int Y_BUTTON = 4;
    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_BUMPER = 6;
    public static final int LEFT_TRIGGER_BUTTON = 7;
    public static final int RIGHT_TRIGGER_BUTTON = 8;
    public static final int PREV_BUTTON = 9;
    public static final int START_BUTTON = 10;
    public static final int LEFT_STICK_BUTTON = 11;
    public static final int RIGHT_STICK_BUTTON = 12;

    // Spark MAX CAN IDs //
    public static final int LEFT_FRONT_DRIVE_MOTOR_ID = 3; // NEO motor
    public static final int RIGHT_FRONT_DRIVE_MOTOR_ID = 2; // NEO motor
    public static final int LEFT_REAR_DRIVE_MOTOR_ID = 11; // NEO motor
    public static final int RIGHT_REAR_DRIVE_MOTOR_ID = 4; // NEO motor
    public static final int INTAKE_BAR_MOTOR_ID = 8; // NEO 550 motor
    public static final int DEPLOY_INTAKE_MOTOR_ID = 6; // NEO motor
    public static final int ELEVATOR_STAGE_1_MOTOR_ID = 7; // NEO motor
    public static final int ELEVATOR_STAGE_2_MOTOR_ID = 9; // NEO 550 motor
    public static final int FINGER_MOTOR_ID = 10; // NEO 550 motor

    // Servo IDs //
    public static final int ELEVATOR_DROP_MOTOR_ID = 0;

    // PWM Ports //
    public static final int LED_PWM_ID = 4;
    
    // DIO (Digital Input/Output) Channels //
    // Example: public static final int RIGHT_ENCODER_CHANNEL_A = 0;
    // Example: public static final int RIGHT_ENCODER_CHANNEL_B = 1;
    // Example: public static final int LEFT_ENCODER_CHANNEL_A = 2;
    // Example: public static final int LEFT_ENCODER_CHANNEL_B = 3;

    // Drivetrain Constants //
    public static final double DRIVE_GEAR_RATIO = 10;
    public static final boolean REVERSE_LEFT_FRONT_MOTOR = true;
    public static final boolean REVERSE_LEFT_BACK_MOTOR = true;
    public static final boolean REVERSE_RIGHT_FRONT_MOTOR = false;
    public static final boolean REVERSE_RIGHT_BACK_MOTOR = false;
    public static final double GYRO_TURN_KP = 0.007; // P (Proportional) constant of a PID loop
    public static final double TRACKED_TAG_ROATION_KP = 0.3; // P (Proportional) constant of a PID loop
    public static final double TRACKED_TAG_FORWARD_DRIVE_KP = 0.4; // P (Proportional) constant of a PID loop
    public static final double TRACKED_TAG_STRAFE_DRIVE_KP = 0.5; // P (Proportional) constant of a PID loop
    public static final double APRILTAG_ROTATION_POWER_CAP = 0.3;
    public static final double APRILTAG_FORWARD_POWER_CAP = 0.3;
    public static final double APRILTAG_STRAFE_POWER_CAP = 0.3;
    public static final double APRILTAG_TRACKING_DISTANCE_THRESHOLD = 0.1;
    public static final double TURNING_THRESHOLD_DEGREES = 3;
    public static final double MAX_POWER_GYRO = 0.4;
    public static final double kP_FRONT_RIGHT_VELOCITY = 0.0010269; // TODO: Update this for 2025
	public static final double kP_FRONT_LEFT_VELOCITY = 0.0010269; // TODO: Update this for 2025
	public static final double kP_BACK_RIGHT_VELOCITY = 0.0010269; // TODO: Update this for 2025
	public static final double kP_BACK_LEFT_VELOCITY = 0.0010269; // TODO: Update this for 2025
	public static final double kP_X_CONTROLLER = 9.6421; // TODO: Update this for 2025
    public static final double kP_Y_CONTROLLER = 9.6421; // TODO: Update this for 2025
    public static final double kP_THETA_CONTROLLER = 9.6421; // TODO: Update this for 2025
	public static final double kMAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2*Math.PI; // TODO: Update this for 2025
	public static final double kMAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 2*Math.PI; // TODO: Update this for 2025

    // Coral Elevator Constants //
    public static final boolean ELEVATOR_INVERT = false;
    public static final double ELEVATOR_SPEED = 0.5; // TODO: Tune this
    public static final double ARM_SPEED = 0.2; // TODO: Tune this
    public static final double WHEEL_SPEED = 0.9; // TODO: Tune this
    public static final boolean ELEVATOR_UP = true;
    public static final boolean ELEVATOR_DOWN = false;
    public static final boolean ARM_UP = true;
    public static final boolean ARM_DOWN = false;
    public static final boolean WHEEL_INTAKE = true;
    public static final boolean WHEEL_OUTTAKE = false;

    public static final int ELEVATOR_ROTATIONS_PER_INCH = 13; // number of rotations elevator climb motor must complete to raise/lower elevator by one inch

    // Intake Constants //
    public static final double DEPLOY_SPEED = 0.1;
    public static final double INTAKE_DEPLOYED_POS = 1.0;
    public static final double INTAKE_RETURNED_POS = 0.01;
    public static final boolean INTAKE_BAR_INVERT = false;
    public static final double INTAKE_BAR_SPEED = 0.8;

    // REV PH Channels //
    // Example: public static final int EXTENSION_SOLENOID_ID = 0;

    // Rev PDH Constants //
    public static final int LEFT_FRONT_DRIVE_MOTOR_PDH_CHANNEL = 11;
    public static final int RIGHT_FRONT_DRIVE_MOTOR_PDH_CHANNEL = 10;
    public static final int LEFT_BACK_DRIVE_MOTOR_PDH_CHANNEL = 12;
    public static final int RIGHT_BACK_DRIVE_MOTOR_PDH_CHANNEL = 13;
    // Example: public static final int LOWER_INTAKE_MOTOR_PDH_CHANNEL = 14;
    // Example: public static final int FLYWHEEL_MOTOR_PDH_CHANNEL = 16;

    // Pneumatics Constants //
    public static final int COMPRESSOR_CAN_ID = 7;

    // Apriltag Vision Constants //
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(7);
    public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(18.5);
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(18);
    public static final String USB_CAMERA_NAME = "Arducam_OV9782_USB_Camera";

}