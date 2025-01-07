// Author: UMN Robotics Ri3D
// Last Updated: December 2024

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.commands.elevator.CoralElevatorSetPositionArmCommand;
import frc.robot.commands.elevator.CoralElevatorSetPositionClimbCommand;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralElevatorSubsystem extends SubsystemBase {

    // Coral Elevator Motor Controllers
    private SparkMax m_elevator_climb_1; // NEO motor
    private SparkMax m_elevator_climb_2; // NEO motor
    private SparkMax m_elevator_arm; // NEO motor
    private SparkMax m_elevator_wheel; // NEO motor

    private double gravityControl;

    // Coral Elevator limiters
    public double climb_max_1 = 10; // TODO: TUNE THIS LIMIT
    public double climb_max_2 = 10; // TODO: TUNE THIS LIMIT
    public double climb_min_1 = 0; // TODO: TUNE THIS LIMIT
    public double climb_min_2 = 0; // TODO: TUNE THIS LIMIT
    public double arm_max = 10; // TODO: TUNE THIS LIMIT
    public double arm_min = 0; // TODO: TUNE THIS LIMIT

    /** Subsystem for controlling the coral elevator */
    public CoralElevatorSubsystem() {
      // Configure the Spark MAX motor controller using the new 2025 method
      m_elevator_climb_1 = new SparkMax(Constants.ELEVATOR_STAGE_1_MOTOR_ID, MotorType.kBrushless);
      configureSparkMAX(m_elevator_climb_1, Constants.ELEVATOR_INVERT);
      m_elevator_climb_2 = new SparkMax(Constants.ELEVATOR_STAGE_2_MOTOR_ID, MotorType.kBrushless);
      configureSparkMAX(m_elevator_climb_2, Constants.ELEVATOR_INVERT);
      m_elevator_arm = new SparkMax(Constants.END_EFFECTOR_ARM_MOTOR_ID, MotorType.kBrushless);
      configureSparkMAX(m_elevator_arm, Constants.ELEVATOR_INVERT);
      m_elevator_wheel = new SparkMax(Constants.END_EFFECTOR_WHEEL_MOTOR_ID, MotorType.kBrushless);
      configureSparkMAX(m_elevator_wheel, Constants.ELEVATOR_INVERT);
  
      // Put the default speed on SmartDashboard if needed
      // SmartDashboard.putNumber("Elevator Speed", Constants.ELEVATOR_SPEED);
    }
  
    private void configureSparkMAX(SparkMax max, boolean reverse) {
      SparkMaxConfig config = new SparkMaxConfig();
      config.inverted(reverse).idleMode(IdleMode.kBrake);
      max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  
  // Climb Motors Methods --------------------------------------------------------------------------------

  /* Sets speed of the elevator CLimb motor one. Inbuilt limiters */
  public void setSpeedClimbOne(double speed) {
    // Spark Max set() method with inbuilt limiters
    if ((speed > 0) && (getPositionClimbOne() > climb_max_1)) {
      m_elevator_climb_1.set(0);
    } else if ((speed < 0) && (getPositionClimbOne() < climb_min_1))  {
      m_elevator_climb_1.set(0);
    } else {
      m_elevator_climb_1.set(speed);
    }
  }
  
  /* Gets position of the elevator climb motor one */
  public double getPositionClimbOne() {
    // Spark Max getEncoder().getPosition() method
    return m_elevator_climb_1.getEncoder().getPosition();
  }

  /* Set climb motor one speed to 0 */
  public void stopClimbOne() {
    setSpeedClimbOne(0);
  }

  public double getGravityControl() {
    return gravityControl;
  }

  /* Sets speed of the elevator CLimb motor two. Inbuilt limiters */
  public void setSpeedClimbTwo(double speed) {
    // Spark Max set() method with inbuilt limiters
    if ((speed > 0) && (getPositionClimbTwo() > climb_max_2)) {
      m_elevator_climb_2.set(0);
    } else if ((speed < 0) && (getPositionClimbTwo() < climb_min_2))  {
      m_elevator_climb_2.set(0);
    } else {
      m_elevator_climb_2.set(speed);
    }
  }
  
  /* Gets position of the elevator climb motor two */
  public double getPositionClimbTwo() {
    // Spark Max getEncoder().getPosition() method
    return m_elevator_climb_2.getEncoder().getPosition();
  }

  /* Set Climb motor two speed to 0 */
  public void stopClimbTwo() {
    setSpeedClimbTwo(0);
  }

  /* Set speed of both climb motors at once */
  public void setSpeedClimb(double speed_1, double speed_2) {
    setSpeedClimbOne(speed_1);
    setSpeedClimbTwo(speed_2);
  }

  /* Set both Climb motor speeds to 0 */
  public void stopClimb() {
    setSpeedClimbOne(0);
    setSpeedClimbTwo(0);
  }

  /* Sets position of elevator climb to Neutral preset */
  public void climbNeutral() {
    // Calls CoralElevatorSetPositionClimbCommand()
    (new CoralElevatorSetPositionClimbCommand(0, 0)).schedule();
  }

  /* Sets position of elevator climb to low Goal preset */
  public void climbLowGoal() {
    // Calls CoralElevatorSetPositionClimbCommand()
    (new CoralElevatorSetPositionClimbCommand(1.5 * Constants.ELEVATOR_ROTATIONS_PER_INCH, 1.5 * Constants.ELEVATOR_ROTATIONS_PER_INCH)).schedule(); // TODO: TUNE THIS
  }

  /* Sets position of elevator climb to Mid Goal preset */
  public void climbMidGoal() {
    // Calls CoralElevatorSetPositionClimbCommand()
    (new CoralElevatorSetPositionClimbCommand(8.5 * Constants.ELEVATOR_ROTATIONS_PER_INCH, 8.5 * Constants.ELEVATOR_ROTATIONS_PER_INCH)).schedule(); // TODO: TUNE THIS
  }

  /* Sets position of elevator climb to High Goal preset */
  public void climbHighGoal() {
    // Calls CoralElevatorSetPositionClimbCommand()
    (new CoralElevatorSetPositionClimbCommand(37 * Constants.ELEVATOR_ROTATIONS_PER_INCH, 37 * Constants.ELEVATOR_ROTATIONS_PER_INCH)).schedule(); // TODO: TUNE THIS
  }

  // Arm Motor Methods -------------------------------------------------------------------------------

  /* Sets speed of the elevator Arm motor. Inbuilt limiters */
  public void setSpeedArm(double speed) {
    // Spark Max set() method with inbuilt limiters
    if ((speed > 0) && (getPositionArm() > arm_max)) {
      m_elevator_arm.set(0);
    } else if ((speed < 0) && (getPositionArm() < arm_min))  {
      m_elevator_arm.set(0);
    } else {
      m_elevator_arm.set(speed);
    }
  }

  /* Gets position of the elevator Arm motor */
  public double getPositionArm() {
    // Spark Max getEncoder().getPosition() method
    return m_elevator_arm.getEncoder().getPosition();
  }

  /* Set Arm speed to 0 */
  public void stopArm() {
    setSpeedArm(0);
  }

  /* Sets position of elevator Arm to Drop preset */
  public void armDrop() {
    // Calls CoralElevatorSetPositionArmCommand()
    (new CoralElevatorSetPositionArmCommand(0.75)).schedule(); // TODO: TUNE THIS
  }

  /* Sets position of elevator Arm to Intake preset */
  public void armPlayerIntake() {
    // Calls CoralElevatorSetPositionArmCommand()
    (new CoralElevatorSetPositionArmCommand(0.4)).schedule(); // TODO: TUNE THIS
  }

  /* Sets position of elevator Arm to Horizontal preset */
  public void armHorizontalIntake() {
    // Calls CoralElevatorSetPositionArmCommand()
    (new CoralElevatorSetPositionArmCommand(0.5)).schedule(); // TODO: TUNE THIS
  }

  /* Sets position of elevator Arm to Vertical preset */
  public void armVertical() {
    // Calls CoralElevatorSetPositionArmCommand()
    (new CoralElevatorSetPositionArmCommand(0.25)).schedule(); // TODO: TUNE THIS
  }

  /* Sets position of elevator Arm to Initial preset */
  public void armInitial() {
    // Calls CoralElevatorSetPositionArmCommand()
    (new CoralElevatorSetPositionArmCommand(0)).schedule(); // TODO: TUNE THIS
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

  /* Sets speed of wheel motor to 0 */
  public void stopWheel() {
    setSpeedWheel(0);
  }

  @Override
  public void periodic() {
    gravityControl = Math.sin(getPositionArm() / 70 * 2 * Math.PI)*Constants.ARM_GRAVITY_CONST;

    // Publish encoder values to SmartDashboard
    SmartDashboard.putNumber("Elevator Climb 1 Position", getPositionClimbOne());
    SmartDashboard.putNumber("Elevator Climb 2 Position", getPositionClimbTwo());
    SmartDashboard.putNumber("Elevator Arm Position", getPositionArm());
    SmartDashboard.putNumber("Elevator Wheel Position", getPositionWheel());
  }
}