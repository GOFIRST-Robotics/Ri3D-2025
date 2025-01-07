// Author: UMN Robotics Ri3D
// Last Updated: January 2025

package frc.robot.subsystems;

import frc.robot.Constants;

import com.studica.frc.AHRS;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Drivetrain ****************************************************************
 * The mecanum drivetrain subsystem of the robot. */
public class DriveSubsystem extends SubsystemBase {
  
	// Drivetrain Motor Controllers
	private static SparkMax m_leftFrontMotor; // NEO motor
	private static SparkMax m_rightFrontMotor; // NEO motor
	private static SparkMax m_leftBackMotor; // NEO motor
	private static SparkMax m_rightBackMotor; // NEO motor

	SlewRateLimiter rightFilter;
	SlewRateLimiter leftFilter;

	private double DRIVE_GEAR_RATIO = Constants.DRIVE_GEAR_RATIO;

	private AHRS navx = new AHRS(AHRS.NavXComType.kUSB1); // Instantiate a NavX Gyroscope connected to a roboRIO USB port

	double leftFrontPositionZero, rightFrontPositionZero, leftBackPositionZero, rightBackPositionZero = 0.0;

	private static final double TRACK_WIDTH = Constants.TRACK_WIDTH;
	private static final double WHEEL_BASE = Constants.WHEEL_BASE;

	private static final SimpleMotorFeedforward kFeedforward = new SimpleMotorFeedforward(0.17472, 2.7572, 0.45109); // kS, kV, kA Characterization Constants
	private static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(Constants.kMAX_ANGULAR_SPEED_RADIANS_PER_SECOND, Constants.kMAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
	private final PIDController frontLeftPIDController = new PIDController(Constants.kP_FRONT_LEFT_VELOCITY, 0, 0);
  	private final PIDController frontRightPIDController = new PIDController(Constants.kP_FRONT_RIGHT_VELOCITY, 0, 0);
  	private final PIDController backLeftPIDController = new PIDController(Constants.kP_BACK_LEFT_VELOCITY, 0, 0);
  	private final PIDController backRightPIDController = new PIDController(Constants.kP_BACK_RIGHT_VELOCITY, 0, 0);

	private static final MecanumDriveKinematics kDriveKinematics =
		new MecanumDriveKinematics(new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), 
								   new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), 
								   new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), 
								   new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

	private static MecanumDrive robotDrive;
	private static MecanumDriveOdometry odometry;

	private RobotConfig config;
  
  /** Subsystem for controlling the Drivetrain and accessing the NavX Gyroscope */
  public DriveSubsystem() {
    // Instantiate the Drivetrain motor controllers
    m_leftFrontMotor = new SparkMax(Constants.LEFT_FRONT_DRIVE_MOTOR_ID, MotorType.kBrushless);
    m_rightFrontMotor = new SparkMax(Constants.RIGHT_FRONT_DRIVE_MOTOR_ID, MotorType.kBrushless);
    m_leftBackMotor = new SparkMax(Constants.LEFT_REAR_DRIVE_MOTOR_ID, MotorType.kBrushless);
    m_rightBackMotor = new SparkMax(Constants.RIGHT_REAR_DRIVE_MOTOR_ID, MotorType.kBrushless);

	robotDrive = new MecanumDrive(m_leftFrontMotor, m_leftBackMotor, m_rightFrontMotor, m_rightBackMotor);
	odometry = new MecanumDriveOdometry(kDriveKinematics, navx.getRotation2d(), getWheelPositions());	

    // Configure the Spark MAX motor controllers using the new 2025 method
    configureSparkMAX(m_leftFrontMotor, Constants.REVERSE_LEFT_FRONT_MOTOR);
    configureSparkMAX(m_leftBackMotor, Constants.REVERSE_LEFT_BACK_MOTOR);
    configureSparkMAX(m_rightBackMotor, Constants.REVERSE_RIGHT_FRONT_MOTOR);
    configureSparkMAX(m_rightFrontMotor, Constants.REVERSE_RIGHT_BACK_MOTOR);

    resetEncoders(); // Zero the drive encoders

    rightFilter = new SlewRateLimiter(5);
    leftFilter = new SlewRateLimiter(5);

	configureAutoBuilder();

    System.out.println("NavX Connected: " + navx.isConnected());
  }

  private void configureAutoBuilder() {
	try {
		config = RobotConfig.fromGUISettings();
	} catch (Exception e) {
		e.printStackTrace();
	}

	AutoBuilder.configure(
	this::getPose, 
	this::resetOdometry, 
	() -> { return kDriveKinematics.toChassisSpeeds(getWheelSpeeds()); }, 
	(speeds, feedforward) -> {
		MecanumDriveWheelSpeeds wheelSpeeds = kDriveKinematics.toWheelSpeeds(speeds);
		setWheelSpeeds(wheelSpeeds);	
	},
	new PPHolonomicDriveController(
		new PIDConstants(3, 0.0, 0.0), 
		new PIDConstants(3, 0.0, 0.0)
	),
	config,
	() -> {
		var alliance = DriverStation.getAlliance();
		if (alliance.isPresent()) {
			return alliance.get() == DriverStation.Alliance.Red;
		}
		return false;
	},
	this);
  }

	private void configureSparkMAX(SparkMax max, boolean reverse) {
		SparkMaxConfig config = new SparkMaxConfig();
		config.inverted(reverse).idleMode(IdleMode.kCoast);
		max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	public void stop() {
		m_leftFrontMotor.set(0);
		m_rightFrontMotor.set(0);
		m_leftBackMotor.set(0);
		m_rightBackMotor.set(0);
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
	public double getGyroAngle() { // Returns the heading of the robot
		return navx.getAngle();
	}
	public double getTurnRate() { // Returns the turn rate of the robot
		return -navx.getRate();
	}

	/** Odometry Methods *******************************************************/
	public Rotation2d getRotation2d() {
		return Rotation2d.fromDegrees(navx.getAngle());
	}
	public void resetOdometry(Pose2d pose) {
		odometry.resetPosition(this.getRotation2d(), getWheelPositions(), pose);
	}
	public Pose2d getPose() {
		return odometry.getPoseMeters();
	}
	public MecanumDriveKinematics getkDriveKinematics() {
		return kDriveKinematics;	
	}
	public TrapezoidProfile.Constraints getkThetaControllerConstraints() {
		return kThetaControllerConstraints;
	}

	@Override
	public void periodic() {
		// Update the odometry in the periodic block
		odometry.update(this.getRotation2d(), getWheelPositions());

		SmartDashboard.putNumber("Left Front Position", getLeftFrontPosition());
		SmartDashboard.putNumber("Right Front Position", getRightFrontPosition());
		SmartDashboard.putNumber("Left Back Position", getLeftBackPosition());
		SmartDashboard.putNumber("Right Back Position", getRightBackPosition());
	}

	//Not Field-Oriented (aka Robot-Oriented)
	public void driveCartesian(double ySpeed, double xSpeed, double zRotation) {
		robotDrive.driveCartesian(ySpeed, xSpeed, zRotation);
	}
	// Field-Oriented
	public void driveCartesian(double ySpeed, double xSpeed, double zRotation, Rotation2d currentAngle) {
		robotDrive.driveCartesian(ySpeed, xSpeed, zRotation, currentAngle);
	}

	/** Get the encoder positions or speeds **************************************/
	public double getLeftFrontPosition() { // Position is returned in units of revolutions
		return (m_leftFrontMotor.getEncoder().getPosition() / DRIVE_GEAR_RATIO - leftFrontPositionZero); // DRIVE_GEAR_RATIO : 1 is our drivetrain gear ratio
	}
	public double getRightFrontPosition() { // Position is returned in units of revolutions
		return (m_rightFrontMotor.getEncoder().getPosition() / DRIVE_GEAR_RATIO - rightFrontPositionZero); // DRIVE_GEAR_RATIO : 1 is our drivetrain gear ratio
	}
	public double getLeftBackPosition() { // Position is returned in units of revolutions
		return (m_leftBackMotor.getEncoder().getPosition() / DRIVE_GEAR_RATIO - leftBackPositionZero); // DRIVE_GEAR_RATIO : 1 is our drivetrain gear ratio
	}
	public double getRightBackPosition() { // Position is returned in units of revolutions
		return (m_rightBackMotor.getEncoder().getPosition() / DRIVE_GEAR_RATIO - rightBackPositionZero); // DRIVE_GEAR_RATIO : 1 is our drivetrain gear ratio
	}
	public double getLeftFrontSpeed() { // Speed is returned in units of RPM (revolutions per minute)
		return (m_leftFrontMotor.getEncoder().getVelocity() / DRIVE_GEAR_RATIO); // DRIVE_GEAR_RATIO : 1 is our drivetrain gear ratio
	}
	public double getRightFrontSpeed() { // Speed is returned in units of RPM (revolutions per minute)
		return (m_rightFrontMotor.getEncoder().getVelocity() / DRIVE_GEAR_RATIO); // DRIVE_GEAR_RATIO : 1 is our drivetrain gear ratio
	}
	public double getLeftBackSpeed() { // Speed is returned in units of RPM (revolutions per minute)
		return (m_leftBackMotor.getEncoder().getVelocity() / DRIVE_GEAR_RATIO); // DRIVE_GEAR_RATIO : 1 is our drivetrain gear ratio
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

	public void setWheelSpeeds(MecanumDriveWheelSpeeds speeds) {
		final double frontLeftFeedforward = kFeedforward.calculate(speeds.frontLeftMetersPerSecond);
		final double frontRightFeedforward = kFeedforward.calculate(speeds.frontRightMetersPerSecond);
		final double backLeftFeedforward = kFeedforward.calculate(speeds.rearLeftMetersPerSecond);
		final double backRightFeedforward = kFeedforward.calculate(speeds.rearRightMetersPerSecond);

		final double frontLeftOutput =
			frontLeftPIDController.calculate(speedToMeters(getLeftFrontSpeed()), speeds.frontLeftMetersPerSecond);
		final double frontRightOutput =
			frontRightPIDController.calculate(speedToMeters(getRightFrontSpeed()), speeds.frontRightMetersPerSecond);
		final double backLeftOutput =
			backLeftPIDController.calculate(speedToMeters(getLeftBackSpeed()), speeds.rearLeftMetersPerSecond);
		final double backRightOutput =
			backRightPIDController.calculate(speedToMeters(getRightBackSpeed()), speeds.rearRightMetersPerSecond);

		m_leftFrontMotor.setVoltage(frontLeftOutput + frontLeftFeedforward);
		m_rightFrontMotor.setVoltage(frontRightOutput + frontRightFeedforward);
		m_leftBackMotor.setVoltage(backLeftOutput + backLeftFeedforward);
		m_rightBackMotor.setVoltage(backRightOutput + backRightFeedforward);
  }

  // Methods for getting the speeds and positions of the drivetrain wheels
  public MecanumDriveWheelPositions getWheelPositions() {
		return new MecanumDriveWheelPositions(positionToMeters(getLeftFrontPosition()), 
											  positionToMeters(getRightFrontPosition()), 
											  positionToMeters(getLeftBackPosition()), 
											  positionToMeters(getRightBackPosition()));
  }
  public MecanumDriveWheelSpeeds getWheelSpeeds() {
		return new MecanumDriveWheelSpeeds(speedToMeters(getLeftFrontSpeed()), 
											speedToMeters(getRightFrontSpeed()), 
											speedToMeters(getLeftBackSpeed()), 
											speedToMeters(getRightBackSpeed()));
  }

	// Conversion Methods: Convert position & speed to Meters
	public double positionToMeters(double position) {
		return position * Math.PI * Constants.WHEEL_DIAMETER;
	}
	public double speedToMeters(double speed) {
		return speed / 60 * Math.PI * Constants.WHEEL_DIAMETER;
	}
}