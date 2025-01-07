// Author: UMN Robotics Ri3D
// Last Updated: January 2025

package frc.robot.commands.autonomous.basic_path_planning;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

import frc.robot.subsystems.DriveSubsystem;

/** Drivetrain Gyro Strafe **************************************************
 * Command for driving sideways using gyroscope feedback. */
public class Drivetrain_GyroStrafe extends Command {
  
  	/** Configuration Constants ***********************************************/
	private static final double kP = Constants.GYRO_TURN_KP;
	private static final double CIRCUMFRENCE = Constants.WHEEL_DIAMETER * Math.PI;
	private static final double MAX_CORRECTION = Constants.MAX_POWER_GYRO;
	
	/** Instance Variables ****************************************************/
	DriveSubsystem drivetrain = Robot.m_driveSubsystem;
	double strafePower, goalAngle, goalDistance;

  	/** Creates a new Drivetrain_GyroStrafe. */
  	public Drivetrain_GyroStrafe(double distance, double power) {
		strafePower = power;
		
		// convert distance to revolutions
		goalDistance = distance / CIRCUMFRENCE;

		addRequirements(drivetrain);
  	}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
		drivetrain.driveCartesian(0, 0, 0);
		goalAngle = drivetrain.getGyroAngle();
		drivetrain.resetEncoders();
  	}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = goalAngle - drivetrain.getGyroAngle();
		
		double correction = kP * error;

		correction = Math.min(MAX_CORRECTION, correction);
		correction = Math.max(-MAX_CORRECTION, correction);
		
		drivetrain.driveCartesian(strafePower, 0, -1 * correction);
 	 }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean rightFrontGoalReached = Math.abs(drivetrain.getRightFrontPosition()) >= goalDistance;
		boolean rightBackGoalReached = Math.abs(drivetrain.getRightBackPosition()) >= goalDistance;
		return rightFrontGoalReached || rightBackGoalReached;
  	}

    // Called once the command ends or is interrupted.
	public void end(boolean interrupted) {
		drivetrain.driveCartesian(0, 0, 0);
	}
}