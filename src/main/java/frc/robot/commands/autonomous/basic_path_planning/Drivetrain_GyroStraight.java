// Author: UMN Robotics Ri3D
// Last Updated: January 2025

package frc.robot.commands.autonomous.basic_path_planning;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

import frc.robot.subsystems.DriveSubsystem;

/** Drivetrain Gyro Straight **************************************************
 * Command for driving straight using gyroscope feedback. */
public class Drivetrain_GyroStraight extends Command {
	/** Configuration Constants ***********************************************/
	private static final double kP = Constants.GYRO_TURN_KP;
	private static final double CIRCUMFRENCE = Constants.WHEEL_DIAMETER * Math.PI;
	private static final double MAX_CORRECTION = Constants.MAX_POWER_GYRO;
	
	/** Instance Variables ****************************************************/
	DriveSubsystem drivetrain = Robot.m_driveSubsystem;
	double forwardPower, goalAngle, goalDistance;
	
	/** Drivetrain Gyro Straight **********************************************
	 * Required subsystems will cancel commands when this command is run.
	 * distance is given in physical units matching the wheel diameter unit
	 * speed is given in physical units per second. The physical units should 
	 * match that of the Wheel diameter. */
	public Drivetrain_GyroStraight(double distance, double power) {
		forwardPower = power;
		
		// convert distance to revolutions
		goalDistance = distance / CIRCUMFRENCE;

		addRequirements(drivetrain);
	}
	
	/** initialize ************************************************************
	 * Called just before this Command runs the first time */
	public void initialize() {
		drivetrain.driveCartesian(0, 0, 0);
		goalAngle = drivetrain.getGyroAngle();
		drivetrain.resetEncoders();
	}

	/** execute ***************************************************************
	 * Called repeatedly when this Command is scheduled to run */
	public void execute() {
		double error = goalAngle - drivetrain.getGyroAngle();
		
		double correction = kP * error;

		correction = Math.min(MAX_CORRECTION, correction);
		correction = Math.max(-MAX_CORRECTION, correction);
		
		drivetrain.driveCartesian(0, forwardPower, -1 * correction);
	}
	
	/** isFinished ************************************************************	
	 * Make this return true when this Command no longer needs to run execute() */
	public boolean isFinished() {
		boolean leftFrontGoalReached = Math.abs(drivetrain.getLeftFrontPosition()) >= goalDistance;
		boolean rightFrontGoalReached = Math.abs(drivetrain.getRightFrontPosition()) >= goalDistance;
		return leftFrontGoalReached || rightFrontGoalReached;
	}

	// Called once the command ends or is interrupted.
	public void end(boolean interrupted) {
		drivetrain.driveCartesian(0, 0, 0);
	}
}