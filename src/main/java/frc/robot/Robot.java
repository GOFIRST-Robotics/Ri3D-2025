// Author: UMN Robotics Ri3D
// Last Updated: January 2025

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.CoralElevatorArmMoveCommand;
import frc.robot.commands.CoralElevatorHorizontalIntakeCommand;
import frc.robot.commands.CoralElevatorMoveCommand;
import frc.robot.commands.CoralElevatorNeutralCommand;
import frc.robot.commands.CoralElevatorScoreHighCommand;
import frc.robot.commands.CoralElevatorScoreLowCommand;
import frc.robot.commands.CoralElevatorScoreMidCommand;
import frc.robot.commands.CoralElevatorWheelMoveCommand;
import frc.robot.commands.DriveToTrackedTargetCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeToggleCommand;
import frc.robot.subsystems.CoralElevatorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PowerSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDMode;
import frc.robot.subsystems.VisionSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  Command m_autonomousCommand;
	SendableChooser<Command> autonChooser = new SendableChooser<Command>(); // Create a chooser to select an autonomous command

  public static boolean manualDriveControl = true;

  public static final GenericHID controller = new GenericHID(Constants.CONTROLLER_USB_PORT_ID); // Instantiate our controller at the specified USB port

  public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem(); // Drivetrain subsystem
  public static final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(); // Intake subsystem
  public static final CoralElevatorSubsystem m_CoralElevatorSubsystem = new CoralElevatorSubsystem(); // Elevator subsystem
  public static final PowerSubsystem m_powerSubsystem = new PowerSubsystem(); // Power subsystem for interacting with the Rev PDH
  public static final VisionSubsystem m_visionSubsystem = new VisionSubsystem(); // Subsystem for interacting with Photonvision
  public static final LEDSubsystem m_LEDSubsystem = new LEDSubsystem(); // Subsytem for controlling the REV Blinkin LED module

  public static final Field2d m_Field = new Field2d();
  
  double goalAngle;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    configureButtonBindings(); // Bind our commands to physical buttons on a controller

    // Add our Autonomous Routines to the chooser //
		autonChooser.setDefaultOption("Do Nothing", new InstantCommand());
    autonChooser.addOption("Test Auto", AutoBuilder.buildAuto("Test Auto"));
		SmartDashboard.putData("Auto Mode", autonChooser);

    // Zero the gyroscope and reset the drive encoders
    m_driveSubsystem.zeroGyro();
    m_driveSubsystem.resetEncoders();
    m_LEDSubsystem.setLEDMode(LEDMode.DISABLED);
    
    SmartDashboard.putData("m_field", m_Field);

    FollowPathCommand.warmupCommand().schedule();
    PathfindingCommand.warmupCommand().schedule();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods. This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Gyroscope Pitch", m_driveSubsystem.getPitch());
    SmartDashboard.putNumber("Gyroscope Yaw", m_driveSubsystem.getYaw());
    SmartDashboard.putNumber("Gyroscope Roll", m_driveSubsystem.getRoll());
    m_Field.setRobotPose(m_driveSubsystem.getPose());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    System.out.println("ROBOT DISABLED");
    m_LEDSubsystem.setLEDMode(LEDMode.DISABLED);
  }

  /** This function is called continuously after the robot enters Disabled mode. */
  @Override
  public void disabledPeriodic() {

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    System.out.println("AUTONOMOUS MODE STARTED");

    m_autonomousCommand = autonChooser.getSelected();
    
    // Zero the gyrodcope and reset the drive encoders
    m_driveSubsystem.zeroGyro();
    m_driveSubsystem.resetEncoders();

    // schedule the selected autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    // Set the LED pattern for autonomous mode
    m_LEDSubsystem.setLEDMode(LEDMode.AUTO);

    // Set Elevator/End Effector inital preset
    m_CoralElevatorSubsystem.climbNeutral();
    m_CoralElevatorSubsystem.armInitial();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    System.out.println("TELEOP MODE STARTED");

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this if statement or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Zero the gyroscope and reset the drive encoders
    m_driveSubsystem.zeroGyro();
    m_driveSubsystem.resetEncoders();

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        // Set the LED pattern for teleop mode
      m_LEDSubsystem.setLEDMode(LEDMode.TELEOPRED);

      }
      if (ally.get() == Alliance.Blue) {
        m_LEDSubsystem.setLEDMode(LEDMode.TELEOPBLUE);
      }
    }

    goalAngle = m_driveSubsystem.getGyroAngle();

    // Set Elevator/End Effector inital preset
    m_CoralElevatorSubsystem.climbNeutral();
    m_CoralElevatorSubsystem.armInitial();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Log controller inputs to SmartDashboard
    // SmartDashboard.putNumber("Controller: Right Trigger", controller.getRawAxis(Constants.RIGHT_TRIGGER_AXIS));
    // SmartDashboard.putNumber("Controller: Left Trigger", controller.getRawAxis(Constants.LEFT_TRIGGER_AXIS));
    // SmartDashboard.putBoolean("Controller: Right Bumper", controller.getRawButton(Constants.RIGHT_BUMPER));
    // SmartDashboard.putBoolean("Controller: Left Bumper", controller.getRawButton(Constants.LEFT_BUMPER));
    // SmartDashboard.putBoolean("Controller: X Button", controller.getRawButton(Constants.X_BUTTON));
    // SmartDashboard.putBoolean("Controller: Y Button", controller.getRawButton(Constants.Y_BUTTON));
    // SmartDashboard.putBoolean("Controller: B Button", controller.getRawButton(Constants.B_BUTTON));
    // SmartDashboard.putBoolean("Controller: A Button", controller.getRawButton(Constants.A_BUTTON));
    // SmartDashboard.putNumber("Controller: Left Joystick X Axis", controller.getRawAxis(Constants.LEFT_HORIZONTAL_JOYSTICK_AXIS));
    // SmartDashboard.putNumber("Controller: Left Joystick Y Axis", controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS));
    // SmartDashboard.putNumber("Controller: Right Joystick X Axis", controller.getRawAxis(Constants.RIGHT_HORIZONTAL_JOYSTICK_AXIS));
    // SmartDashboard.putNumber("Controller: Right Joystick Y Axis", controller.getRawAxis(Constants.RIGHT_VERTICAL_JOYSTICK_AXIS));

  if (Robot.manualDriveControl) {
    double ySpeed = -controller.getRawAxis(Constants.RIGHT_VERTICAL_JOYSTICK_AXIS);
    double xSpeed = controller.getRawAxis(Constants.RIGHT_HORIZONTAL_JOYSTICK_AXIS);
    double zSpeed = controller.getRawAxis(Constants.LEFT_HORIZONTAL_JOYSTICK_AXIS);
    
    // Speed limits
    ySpeed = Math.max(Math.min(ySpeed, 0.4), -0.4);
    xSpeed = Math.max(Math.min(xSpeed, 0.4), -0.4);
    zSpeed = Math.max(Math.min(zSpeed, 0.4), -0.4);

    if (Math.abs(zSpeed) > 0.01) { // If we are telling the robot to rotate, then let it rotate
			// m_driveSubsystem.driveCartesian(ySpeed, xSpeed, zSpeed, m_driveSubsystem.getRotation2d()); // field-relative
      m_driveSubsystem.driveCartesian(ySpeed, xSpeed, zSpeed); // robot-relative
			goalAngle = m_driveSubsystem.getGyroAngle();
		}
		else { // Otherwise, use the gyro to maintain our current angle
			double error = m_driveSubsystem.getGyroAngle() - goalAngle;
			
			double correction = Constants.GYRO_TURN_KP * error;
      if (Math.abs(correction) > Constants.MAX_POWER_GYRO) { // Maximum value we want
        correction = Math.copySign(Constants.MAX_POWER_GYRO, correction);
      }
			
			// m_driveSubsystem.driveCartesian(ySpeed, xSpeed, -1 * correction, m_driveSubsystem.getRotation2d()); // field-relative
      m_driveSubsystem.driveCartesian(ySpeed, xSpeed, correction); // robot-relative
      }
    } else {
      goalAngle = m_driveSubsystem.getGyroAngle();
		}
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    System.out.println("TEST MODE STARTED");
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or onse of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} 
   * or {@link XboxController}), and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.Trigger}.
   */
  private void configureButtonBindings() {
    // Intake Controls //
    new Trigger(() -> controller.getRawButton(Constants.RIGHT_BUMPER)).whileTrue(new IntakeCommand()); // Intake or Outake depending on position
    new Trigger(() -> controller.getRawButton(Constants.B_BUTTON)).onTrue(new IntakeToggleCommand()); // Deploy or Retract Intake

    // Coral Elevator Controls //
    new Trigger(() -> controller.getRawButton(Constants.RIGHT_BUMPER)).whileTrue(new CoralElevatorMoveCommand(Constants.ELEVATOR_UP)); // Elevator Up Manual
    new Trigger(() -> controller.getRawButton(Constants.LEFT_BUMPER)).whileTrue(new CoralElevatorMoveCommand(Constants.ELEVATOR_DOWN)); // Elevator Down Manual
    new Trigger(() -> controller.getRawButton(Constants.RIGHT_STICK_BUTTON)).whileTrue(new CoralElevatorArmMoveCommand(Constants.ARM_UP)); // Arm Up Manual
    new Trigger(() -> controller.getRawButton(Constants.LEFT_STICK_BUTTON)).whileTrue(new CoralElevatorArmMoveCommand(Constants.ARM_DOWN)); // Arm Down Manual
    new Trigger(() -> controller.getRawButton(Constants.PREV_BUTTON)).whileTrue(new CoralElevatorWheelMoveCommand(Constants.WHEEL_OUTTAKE)); // Wheel Outtake Manual
    new Trigger(() -> controller.getRawButton(Constants.START_BUTTON)).whileTrue(new CoralElevatorWheelMoveCommand(Constants.WHEEL_INTAKE)); // Weel Intake Manual
    new Trigger(() -> controller.getRawButton(Constants.A_BUTTON)).onTrue(new CoralElevatorNeutralCommand()); // Neutral Preset
    new POVButton(controller, 0).whileTrue(new CoralElevatorScoreMidCommand()); // Score Mid Preset
    new POVButton(controller, 90).whileTrue(new CoralElevatorScoreHighCommand()); // Score High Preset
    new POVButton(controller, 180).whileTrue(new CoralElevatorHorizontalIntakeCommand()); // Horizontal Intake Preset
    new POVButton(controller, 270).whileTrue(new CoralElevatorScoreLowCommand()); // Score Low Preset

    // Test Controls //
    new Trigger(() -> controller.getRawButton(Constants.A_BUTTON)).whileTrue(new DriveToTrackedTargetCommand(0.75)); // Track AprilTag
  }
}