// Author: UMN Robotics Ri3D
// Last Updated: December 2024

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.commands.CoralElevatorSetPositionClimbCommand;
import frc.robot.commands.CoralElevatorSetPositionLowerCommand;

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

  /* Sets position of elevator climb to Neutral preset */
  public void climbNeutral() {
    // Calls CoralElevatorSetPositionClimbCommand()
    (new CoralElevatorSetPositionClimbCommand(0)).schedule(); // TODO: TUNE THIS
  }

  /* Sets position of elevator climb to Player Intake preset */
  public void climbPlayerIntake() {
    // Calls CoralElevatorSetPositionClimbCommand()
    (new CoralElevatorSetPositionClimbCommand(2* 12 * Constants.ELEVATOR_ROTATIONS_PER_INCH)).schedule(); // TODO: TUNE THIS
  }

  /* Sets position of elevator climb to low Goal preset */
  public void climbLowGoal() {
    // Calls CoralElevatorSetPositionClimbCommand()
    (new CoralElevatorSetPositionClimbCommand(4* 12 * Constants.ELEVATOR_ROTATIONS_PER_INCH)).schedule(); // TODO: TUNE THIS
  }

  /* Sets position of elevator climb to Mid Goal preset */
  public void climbMidGoal() {
    // Calls CoralElevatorSetPositionClimbCommand()
    (new CoralElevatorSetPositionClimbCommand(6* 12 * Constants.ELEVATOR_ROTATIONS_PER_INCH)).schedule(); // TODO: TUNE THIS
  }

  /* Sets position of elevator climb to High Goal preset */
  public void climbHighGoal() {
    // Calls CoralElevatorSetPositionClimbCommand()
    (new CoralElevatorSetPositionClimbCommand(8* 12 * Constants.ELEVATOR_ROTATIONS_PER_INCH)).schedule(); // TODO: TUNE THIS
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

  /* Sets position of elevator Lower to Drop preset */
  public void lowerDrop() {
    // Calls CoralElevatorSetPositionLowerCommand()
    (new CoralElevatorSetPositionLowerCommand(0.25)).schedule(); // TODO: TUNE THIS
  }

  /* Sets position of elevator Lower to Intake preset */
  public void lowerIntake() {
    // Calls CoralElevatorSetPositionLowerCommand()
    (new CoralElevatorSetPositionLowerCommand(0)).schedule(); // TODO: TUNE THIS
  }

  /* Sets position of elevator Lower to Horizontal preset */
  public void lowerHorizontal() {
    // Calls CoralElevatorSetPositionLowerCommand()
    (new CoralElevatorSetPositionLowerCommand(0.125)).schedule(); // TODO: TUNE THIS
  }

  /* Sets position of elevator Lower to Vertical preset */
  public void lowerVertical() {
    // Calls CoralElevatorSetPositionLowerCommand()
    (new CoralElevatorSetPositionLowerCommand(-0.125)).schedule(); // TODO: TUNE THIS
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