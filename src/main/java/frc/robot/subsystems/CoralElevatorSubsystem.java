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
    private SparkMax m_elevator_lower; // NEO motor
    private Servo m_elevator_finger; // Servo motor
    // TODO: Implement SetPosition for Climb motor
    // TODO: Implement preset heights for Climb motor
  
    /** Subsystem for controlling the coral elevator */
    public CoralElevatorSubsystem() {
      // Configure the Spark MAX motor controller using the new 2025 method
      m_elevator_climb = new SparkMax(Constants.ELEVATOR_STAGE_1_MOTOR_ID, MotorType.kBrushless);
      configureSparkMAX(m_elevator_climb, Constants.ELEVATOR_INVERT);
      m_elevator_lower = new SparkMax(Constants.ELEVATOR_STAGE_2_MOTOR_ID, MotorType.kBrushless);
      configureSparkMAX(m_elevator_climb, Constants.ELEVATOR_INVERT);
      
      m_elevator_finger = new Servo(Constants.ELEVATOR_DROP_MOTOR_ID);
  
      // Put the default speed on SmartDashboard if needed
      // SmartDashboard.putNumber("Elevator Speed", Constants.ELEVATOR_SPEED);
    }
  
    private void configureSparkMAX(SparkMax max, boolean reverse) {
      SparkMaxConfig config = new SparkMaxConfig();
      config.inverted(reverse).idleMode(IdleMode.kBrake);
      max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  
  // Climb Motor Methods --------------------------------------------------------------------------------

  /* Set speed of the elevator climb */
  public void setSpeedClimb(double speed) {
    // Spark Max .set() method
    m_elevator_climb.set(speed);
  }
  
  /* Gets position of the elevator climb motor */
  public double getPositionClimb() {
      // Spark Max getEncoder().getPosition() method
      return m_elevator_climb.getEncoder().getPosition();
  }

  // Lower Motor Methods -------------------------------------------------------------------------------

  /* Sets speed of the elevator lowering motor */
  public void setSpeedLower(double speed) {
    // Spark Max set() method
    m_elevator_lower.set(speed);
  }

  /* Gets position of the elevator lowering motor */
  public double getPositionLower() {
    // Spark Max getEncoder().getPosition() method
    return m_elevator_lower.getEncoder().getPosition();
  }

  // Finger Motor Methods ------------------------------------------------------------------------------

  /* Set position to the elevator drop motor. Input between 0.0 and 1.0 */
  public void setPositionFinger(double pos) {
    // Servo set method
    m_elevator_finger.set(pos);
  }

  /* Gets current position of elevator drop motor. Output between 0.0 and 1.0 */
  public double getPositionFinger() {
    // Servo get method
    return m_elevator_finger.get();
  }

  /* Set position of the elevator drop motor in terms of an angle */
  public void setAngleFinger(double angle) {
    // Servo setAngle method
    m_elevator_finger.setAngle(angle);
  }

  /* Get position of the elevator drop motor in terms of an angle */
  public double getAngleFinger() {
    // Servo getAngle method
    return m_elevator_finger.getAngle();
  }

  @Override
  public void periodic() {}
}