// Author: UMN Robotics Ri3D
// Last Updated: December 2024

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralElevatorSubsystem extends SubsystemBase {
  
  // Coral Elevator Motor Controllers
  private SparkMax m_elevator_climb; // NEO motor
  private Servo m_elevator_drop; // Servo motor

  /** Subsystem for controlling the coral elevator */
  public CoralElevatorSubsystem() {
    // Configure the Spark MAX motor controller using the new 2025 method
    m_elevator_climb = new SparkMax(Constants.ELEVATOR_CLIMB_MOTOR_ID_SPARK, MotorType.kBrushless);
    configureSparkMAX(m_elevator_climb, Constants.ELEVATOR_INVERT);
    
    m_elevator_drop = new Servo(Constants.ELEVATOR_DROP_MOTOR_ID);

    // Put the default speed on SmartDashboard if needed
    // SmartDashboard.putNumber("Elevator Speed", Constants.ELEVATOR_SPEED);
  }

  private void configureSparkMAX(SparkMax max, boolean reverse) {
		SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(reverse).idleMode(IdleMode.kBrake);
    max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

  /* Set position of the elevator climb motor */
  public void setPositionClimb(double pos) {
    // Spark Max getEncoder().getPosition() method
    m_elevator_climb.getEncoder().setPosition(pos);
  }

  /* Gets position of the elevator climb motor */
  public double getPositionClimb() {
    // Spark Max getEncoder().getPosition() method
    return m_elevator_climb.getEncoder().getPosition();
  }

  /* Set position to the elevator drop motor. Input between 0.0 and 1.0 */
  public void setPos(double pos) {
    // Servo set method
    m_elevator_drop.set(pos);
  }

  /* Gets current position of elevator drop motor. Output between 0.0 and 1.0 */
  public double getPos() {
    // Servo get method
    return m_elevator_drop.get();
  }

  /* Set position of the elevator drop motor in terms of an angle */
  public void setAngle(double angle) {
    // Servo setAngle method
    m_elevator_drop.setAngle(angle);
  }

  /* Get position of the elevator drop motor in terms of an angle */
  public double getAngle() {
    // Servo getAngle method
    return m_elevator_drop.getAngle();
  }

  /* Resets climb and drop motors positions to initial (0) */
  public void reset() {
    setPositionClimb(0);
    setAngle(0);
  }

  @Override
  public void periodic() {}
}