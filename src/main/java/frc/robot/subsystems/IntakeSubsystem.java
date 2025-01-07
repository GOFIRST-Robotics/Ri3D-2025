// Author: UMN Robotics Ri3D
// Last Updated: January 2025

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {  
  // Intake Motor Controllers
  private SparkMax m_IntakeBar; // NEO 550 motor
  private SparkMax m_IntakeArm; // NEO 550 motor

  private double intakeBarRPM; // Stores the rpm of the bar
  private double intakeArmPosition; // Stores the position of the arm

  private double intakeCurrentSum; // Stores the decaying sum of the current

  private double intakeGravityControl;

  /** Subsystem for controlling the Intake */
  public IntakeSubsystem() {
    // Instantiate the Intake motor controllers
    m_IntakeBar = new SparkMax(Constants.INTAKE_BAR_MOTOR_ID, MotorType.kBrushless);
    m_IntakeArm = new SparkMax(Constants.INTAKE_ARM_MOTOR_ID, MotorType.kBrushless);
    
    // Configure the Spark MAX motor controllers using the new 2025 method
    configureSparkMAX(m_IntakeBar, Constants.INTAKE_BAR_INVERT);
    configureSparkMAX(m_IntakeArm, false);
    SmartDashboard.putNumber("Intake Bar Speed", Constants.INTAKE_BAR_SPEED);
  }

  private void configureSparkMAX(SparkMax max, boolean reverse) {
		SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(reverse).idleMode(IdleMode.kBrake);
    max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

  @Override
  public void periodic() {
    intakeBarRPM = m_IntakeBar.getEncoder().getVelocity();
    intakeArmPosition = m_IntakeArm.getEncoder().getPosition();

    intakeCurrentSum = intakeCurrentSum * 0.75 + (m_IntakeBar.getOutputCurrent() * 0.75 + m_IntakeArm.getOutputCurrent() * 0.25) * 0.25;

    intakeGravityControl = intakeArmPosition * Constants.GRAVITY_RESISTANCE / Constants.INTAKE_DEPLOY_LIMIT;

    // Add intake bar RPM and deploy position readings to SmartDashboard for the sake of data logging
    SmartDashboard.putNumber("Intake Bar RPM", intakeBarRPM);
    SmartDashboard.putNumber("Intake Arm Position", intakeArmPosition);
  }

  /* Set power to the intake motor */
  public void setIntakeBarPower(double power) {
    m_IntakeBar.set(power);
  }
  public void stopIntakeBar() {
    m_IntakeBar.set(0);
  }

  /* Read the speed of the intake motor */
  public double getIntakeBarRPM() {
    return intakeBarRPM;
  }
  public double getIntakeGravityControl() {
    return intakeGravityControl;
  }
  public double getIntakeCurrentSum() {
    return intakeCurrentSum;
  }

  /* get position */
  public double getIntakeArmPosition() {
    return intakeArmPosition;
  }

  public void setIntakeArmPower(double power) {
    if(!(power > 0 && getIntakeArmPosition() > Constants.INTAKE_DEPLOY_LIMIT || (power < 0 && getIntakeArmPosition() < Constants.INTAKE_RETURN_LIMIT))) {
      m_IntakeArm.set(power);
    } else {
      m_IntakeArm.set(0);
    }
  }

  public void stopIntakeArm() {
    setIntakeArmPower(0);
  }
}