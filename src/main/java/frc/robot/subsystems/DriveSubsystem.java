// Author: UMN Robotics Ri3D
// Last Updated: December 2024

package frc.robot.subsystems;

import frc.robot.Constants;

import com.studica.frc.AHRS;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  
  // Drivetrain Motor Controllers
  private SparkMax m_leftFrontMotor; // NEO motor
  private SparkMax m_rightFrontMotor; // NEO motor
  private SparkMax m_leftBackMotor; // NEO motor
  private SparkMax m_rightBackMotor; // NEO motor

  // These objects help with fancy path following
  private DifferentialDriveOdometry odometry;
  private DifferentialDriveKinematics kinematics;

  SlewRateLimiter rightFilter;
  SlewRateLimiter leftFilter;

  private double DRIVE_GEAR_RATIO = Constants.DRIVE_GEAR_RATIO;
  private int direction = 1; // This variable is here because we wanted to be able to flip which side of the robot is the 'front'
  
  private AHRS navx = new AHRS(AHRS.NavXComType.kUSB1); // Instantiate a NavX Gyroscope connected to a roboRIO USB port

  double leftFrontPositionZero, rightFrontPositionZero, leftBackPositionZero, rightBackPositionZero = 0.0;

  // Create a chooser for selecting the desired drive speed scale
  SendableChooser<Double> driveScaleChooser = new SendableChooser<Double>();
  public double CURRENT_DRIVE_SCALE;

  /** Subsystem for controlling the Drivetrain and accessing the NavX Gyroscope */
  public DriveSubsystem() {
    // Instantiate the Drivetrain motor controllers
    m_leftFrontMotor = new SparkMax(Constants.LEFT_FRONT_DRIVE_MOTOR_ID, MotorType.kBrushless);
    m_rightFrontMotor = new SparkMax(Constants.RIGHT_FRONT_DRIVE_MOTOR_ID, MotorType.kBrushless);
    m_leftBackMotor = new SparkMax(Constants.LEFT_REAR_DRIVE_MOTOR_ID, MotorType.kBrushless);
    m_rightBackMotor = new SparkMax(Constants.RIGHT_REAR_DRIVE_MOTOR_ID, MotorType.kBrushless);

    // Configure the Spark MAX motor controllers using the new 2025 method
    configureSparkMAX(m_leftFrontMotor, Constants.DRIVE_INVERT_LEFT);
    configureSparkMAX(m_leftBackMotor, Constants.DRIVE_INVERT_LEFT);
    configureSparkMAX(m_rightBackMotor, Constants.DRIVE_INVERT_RIGHT);
    configureSparkMAX(m_rightFrontMotor, Constants.DRIVE_INVERT_RIGHT);

    resetEncoders(); // Zero the drive encoders

    rightFilter = new SlewRateLimiter(5);
    leftFilter = new SlewRateLimiter(5);

    // These objects help with fancy path following
    odometry = new DifferentialDriveOdometry(getRotation2D(), getLeftDistance(), getRightDistance());
    kinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH);

    // Drive Scale Options //
    driveScaleChooser.addOption("100%", 1.0);
    driveScaleChooser.setDefaultOption("75%", 0.75);
    driveScaleChooser.addOption("50%", 0.5);
    driveScaleChooser.addOption("25%", 0.25);

    SmartDashboard.putData("Drivetrain Speed", driveScaleChooser);
    SmartDashboard.putNumber("Left Front Power Pct", 0);
    SmartDashboard.putNumber("Left Back Power Pct", 0);
    SmartDashboard.putNumber("Right Front Power Pct", 0);
    SmartDashboard.putNumber("Right Back Power Pct", 0);

    System.out.println("NavX Connected: " + navx.isConnected());
  }

  private void configureSparkMAX(SparkMax max, boolean reverse) {
		SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(reverse).idleMode(IdleMode.kBrake);
    max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

  /* Set power to the drivetrain motors */
  public void drive(double leftPercentPower, double rightPercentPower) {
    leftPercentPower = leftFilter.calculate(leftPercentPower);
    rightPercentPower = rightFilter.calculate(rightPercentPower);

    m_leftFrontMotor.set(direction * leftPercentPower);
    m_leftBackMotor.set(direction * leftPercentPower);
    m_rightFrontMotor.set(direction * rightPercentPower);
    m_rightBackMotor.set(direction * rightPercentPower);
  }
  public void stop() {
    drive(0, 0);
  }

  // NavX Gyroscope Methods //
  public void zeroGyro() {
    navx.reset();
  }
  public double getYaw() {
    return navx.getYaw();
  }
  public double getPitch() {
    return navx.getPitch();
  }
  public double getRoll() {
    return navx.getRoll();
  }
  public double getAngle() {
    return navx.getAngle();
  }
  public Rotation2d getRotation2D() {
    return navx.getRotation2d();
  }

  /** Get the encoder positions or speeds **************************************/
  public double getLeftFrontPosition() { // Position is returned in units of revolutions
    return (m_leftFrontMotor.getEncoder().getPosition() / DRIVE_GEAR_RATIO - leftFrontPositionZero); // DRIVE_GEAR_RATIO : 1 is our drivetrain gear ratio
  }
  public double getRightFrontPosition() { // Position is returned in units of revolutions
    return -1 * (m_rightFrontMotor.getEncoder().getPosition() / DRIVE_GEAR_RATIO - rightFrontPositionZero); // DRIVE_GEAR_RATIO : 1 is our drivetrain gear ratio
  }
  public double getLeftBackPosition() { // Position is returned in units of revolutions
    return -1 * (m_leftBackMotor.getEncoder().getPosition() / DRIVE_GEAR_RATIO - leftBackPositionZero); // DRIVE_GEAR_RATIO : 1 is our drivetrain gear ratio
  }
  public double getRightBackPosition() { // Position is returned in units of revolutions
    return (m_rightBackMotor.getEncoder().getPosition() / DRIVE_GEAR_RATIO - rightBackPositionZero); // DRIVE_GEAR_RATIO : 1 is our drivetrain gear ratio
  }
  public double getLeftFrontSpeed() { // Speed is returned in units of RPM (revolutions per minute)
    return (m_leftFrontMotor.getEncoder().getVelocity() / DRIVE_GEAR_RATIO); // DRIVE_GEAR_RATIO : 1 is our drivetrain gear ratio
  }
  public double getRightFrontSpeed() { // Speed is returned in units of RPM (revolutions per minute)
    return -1 * (m_rightFrontMotor.getEncoder().getVelocity() / DRIVE_GEAR_RATIO); // DRIVE_GEAR_RATIO : 1 is our drivetrain gear ratio
  }
  public double getLeftBackSpeed() { // Speed is returned in units of RPM (revolutions per minute)
    return -1 * (m_leftBackMotor.getEncoder().getVelocity() / DRIVE_GEAR_RATIO); // DRIVE_GEAR_RATIO : 1 is our drivetrain gear ratio
  }
  public double getRightBackSpeed() { // Speed is returned in units of RPM (revolutions per minute)
    return (m_rightBackMotor.getEncoder().getVelocity() / DRIVE_GEAR_RATIO); // DRIVE_GEAR_RATIO : 1 is our drivetrain gear ratio
  }

  // Zero the drivetrain encoders
  public void resetEncoders() {
		leftFrontPositionZero = m_leftFrontMotor.getEncoder().getPosition() / DRIVE_GEAR_RATIO; // DRIVE_GEAR_RATIO : 1 is our drivetrain gear ratio
		leftBackPositionZero = m_leftBackMotor.getEncoder().getPosition() / DRIVE_GEAR_RATIO; // DRIVE_GEAR_RATIO : 1 is our drivetrain gear ratio
		rightFrontPositionZero = m_rightFrontMotor.getEncoder().getPosition() / DRIVE_GEAR_RATIO; // DRIVE_GEAR_RATIO : 1 is our drivetrain gear ratio
		rightBackPositionZero = m_rightBackMotor.getEncoder().getPosition() / DRIVE_GEAR_RATIO; // DRIVE_GEAR_RATIO : 1 is our drivetrain gear ratio
	}

  // Speed will be measured in meters/second
  public double getLeftSpeed() {
    return speedToMeters(getLeftFrontSpeed()) + speedToMeters(getLeftBackSpeed()) / 2;
  }
  public double getRightSpeed() {
    return speedToMeters(getRightFrontSpeed()) + speedToMeters(getRightBackSpeed()) / 2;
  }
  public double getAverageEncoderSpeed() {
    return (getLeftSpeed() + getRightSpeed()) / 2;
  }

  // Distance will be measured in meters
  public double getLeftDistance() {
    return positionToMeters(getLeftFrontPosition()) + positionToMeters(getLeftBackPosition()) / 2;
  }
  public double getRightDistance() {
    return positionToMeters(getRightFrontPosition()) + positionToMeters(getRightBackPosition()) / 2;
  }
  public double getAverageEncoderDistance() {
    return (getLeftDistance() + getRightDistance()) / 2;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public void updateOdometry() {
    odometry.update(getRotation2D(), getLeftDistance(), getRightDistance());
  }

  // Returns the current wheel speeds of the robot.
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
  }
  // Returns the current wheel positions of the robot.
  public DifferentialDriveWheelPositions getWheelPositions() {
    return new DifferentialDriveWheelPositions(getLeftDistance(), getRightDistance());
  }

  // These methods allow us to flip which side is considered the "front" of the robot
  public void setDirection(DIRECTION direction) {
    this.direction = direction.direction;
  }

  public void toggleDirection() {
    this.direction *= -1;
  }

  @Override
  public void periodic() {
    updateOdometry(); // Update our position on the field

    CURRENT_DRIVE_SCALE = driveScaleChooser.getSelected(); // Continously update the desired drive scale
  }

  // Helps with flipping which side is considered the "front" of the robot
  enum DIRECTION {
    INTAKE_FRONT(1);

    public int direction;

    DIRECTION(int direction) {
      this.direction = direction;
    }
  }

  // Conversion Methods: Convert position & speed to Meters
	public double positionToMeters(double position) {
		return position * Math.PI * Constants.WHEEL_DIAMETER;
	}
	public double speedToMeters(double speed) {
		return speed / 60 * Math.PI * Constants.WHEEL_DIAMETER;
	}
}