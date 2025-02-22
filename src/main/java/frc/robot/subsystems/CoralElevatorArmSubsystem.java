// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.commands.elevator.CoralElevatorSetPositionArmCommand;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralElevatorArmSubsystem extends SubsystemBase {
  private SparkMax m_elevator_arm; // NEO motor

  private double gravityControl;

  public double arm_max = 39.4;
  public double arm_min = 0;

  public Boolean Elevator=false;

  /** Creates a new CoralElevatorArmSubsystem. */
  public CoralElevatorArmSubsystem() {
    m_elevator_arm = new SparkMax(Constants.END_EFFECTOR_ARM_MOTOR_ID, MotorType.kBrushless);
    configureSparkMAX(m_elevator_arm, Constants.ELEVATOR_ARM_INVERT);
  }

  private void configureSparkMAX(SparkMax max, boolean reverse) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(reverse).idleMode(IdleMode.kBrake);
    max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /* Sets speed of the elevator Arm motor. Inbuilt limiters */
  public void setSpeedArm(double speed) {
    if(!Elevator){
      if ((speed > 0) && (getPositionArm() > arm_max)) {
        m_elevator_arm.set(0);
      } else if ((speed < 0) && (getPositionArm() < arm_min))  {
        m_elevator_arm.set(0);
      } else {
        m_elevator_arm.set(speed);
      }
    }
  }

  public void setSpeedArmElevator(double speed) {
    // Spark Max set() method with inbuilt limiters
    if ((speed > 0) && (getPositionArm() > arm_max)) {
      m_elevator_arm.set(0);
    } else if ((speed < 0) && (getPositionArm() < arm_min))  {
      m_elevator_arm.set(0);
    } else {
      m_elevator_arm.set(speed);
    }
  }

  public double getGravityControl() {
    return gravityControl;
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
    (new CoralElevatorSetPositionArmCommand(arm_max)).schedule();
  }

  /* Sets position of elevator Arm to Intake preset */
  public void armPlayerIntake() {
    // Calls CoralElevatorSetPositionArmCommand()
    (new CoralElevatorSetPositionArmCommand(29.2)).schedule();
  }

  /* Sets position of elevator Arm to Vertical preset */
  public void armVertical() {
    // Calls CoralElevatorSetPositionArmCommand()
    (new CoralElevatorSetPositionArmCommand(17.5)).schedule();
  }

  /* Sets position of elevator Arm to Initial preset */
  public void armInitial() {
    // Calls CoralElevatorSetPositionArmCommand()
    (new CoralElevatorSetPositionArmCommand(arm_min)).schedule();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    gravityControl = Math.sin((getPositionArm() / 70 * 2 * Math.PI) + Math.PI/2)*Constants.ARM_GRAVITY_CONST;

    SmartDashboard.putNumber("Elevator Arm Position", getPositionArm());
  }
}
