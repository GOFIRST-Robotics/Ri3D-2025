// Author: UMN Robotics Ri3D
// Last Updated: December 2024

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.commands.CoralElevatorSetPositionClimbCommand;
import frc.robot.commands.CoralElevatorSetPositionArmCommand;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralElevatorSubsystem extends SubsystemBase {

    // Coral Elevator Motor Controllers
    private SparkMax m_elevator_climb; // NEO motor
    private SparkMax m_elevator_arm; // NEO motor
    private SparkMax m_elevator_wheel; // NEO motor

    /** Subsystem for controlling the coral elevator */
    public CoralElevatorSubsystem() {
      // Configure the Spark MAX motor controller using the new 2025 method
      m_elevator_climb = new SparkMax(Constants.ELEVATOR_STAGE_1_MOTOR_ID, MotorType.kBrushless);
      configureSparkMAX(m_elevator_climb, Constants.ELEVATOR_INVERT);
      m_elevator_arm = new SparkMax(Constants.ELEVATOR_STAGE_2_MOTOR_ID, MotorType.kBrushless);
      configureSparkMAX(m_elevator_climb, Constants.ELEVATOR_INVERT);
      m_elevator_wheel = new SparkMax(Constants.ELEVATOR_STAGE_2_MOTOR_ID, MotorType.kBrushless);
      configureSparkMAX(m_elevator_climb, Constants.ELEVATOR_INVERT);
  
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
    (new CoralElevatorSetPositionClimbCommand(0)).schedule();
  }

  // OBSOLETE, USAGE REPLACED BY "NEUTRAL"
  // MAY USE AGAIN IN THE FUTURE
  /* Sets position of elevator climb to Player Intake preset */
  // public void climbPlayerIntake() {
    // Calls CoralElevatorSetPositionClimbCommand()
    // (new CoralElevatorSetPositionClimbCommand(2 * 12 * Constants.ELEVATOR_ROTATIONS_PER_INCH)).schedule();
  // }

  /* Sets position of elevator climb to low Goal preset */
  public void climbLowGoal() {
    // Calls CoralElevatorSetPositionClimbCommand()
    (new CoralElevatorSetPositionClimbCommand(3 * Constants.ELEVATOR_ROTATIONS_PER_INCH)).schedule(); // TODO: TUNE THIS
  }

  /* Sets position of elevator climb to Mid Goal preset */
  public void climbMidGoal() {
    // Calls CoralElevatorSetPositionClimbCommand()
    (new CoralElevatorSetPositionClimbCommand(17 * Constants.ELEVATOR_ROTATIONS_PER_INCH)).schedule(); // TODO: TUNE THIS
  }

  /* Sets position of elevator climb to High Goal preset */
  public void climbHighGoal() {
    // Calls CoralElevatorSetPositionClimbCommand()
    (new CoralElevatorSetPositionClimbCommand(74 * Constants.ELEVATOR_ROTATIONS_PER_INCH)).schedule(); // TODO: TUNE THIS
  }

  // Arm Motor Methods -------------------------------------------------------------------------------

  /* Sets speed of the elevator Arm motor */
  public void setSpeedArm(double speed) {
    // Spark Max set() method
    m_elevator_arm.set(speed);
  }

  /* Gets position of the elevator Arm motor */
  public double getPositionArm() {
    // Spark Max getEncoder().getPosition() method
    return m_elevator_arm.getEncoder().getPosition();
  }

  /* Sets position of elevator Arm to Drop preset */
  public void armDrop() {
    // Calls CoralElevatorSetPositionArmCommand()
    (new CoralElevatorSetPositionArmCommand(0.25)).schedule(); // TODO: TUNE THIS
  }

  /* Sets position of elevator Arm to Intake preset */
  public void armPlayerIntake() {
    // Calls CoralElevatorSetPositionArmCommand()
    (new CoralElevatorSetPositionArmCommand(0)).schedule(); // TODO: TUNE THIS
  }

  /* Sets position of elevator Arm to Horizontal preset */
  public void armHorizontalIntake() {
    // Calls CoralElevatorSetPositionArmCommand()
    (new CoralElevatorSetPositionArmCommand(0.125)).schedule(); // TODO: TUNE THIS
  }

  /* Sets position of elevator Arm to Vertical preset */
  public void armVertical() {
    // Calls CoralElevatorSetPositionArmCommand()
    (new CoralElevatorSetPositionArmCommand(-0.125)).schedule(); // TODO: TUNE THIS
  }

  // Wheel Motor Methods ------------------------------------------------------------------------------

  /* Sets speed of the elevator Wheel motor */
  public void setSpeedWheel(double speed) {
    // Spark Max set() method
    m_elevator_wheel.set(speed);
  }

  /* Gets position of the elevator Wheel motor */
  public double getPositionWheel() {
    // Spark Max getEncoder().getPosition() method
    return m_elevator_wheel.getEncoder().getPosition();
  }

  @Override
  public void periodic() {}
}