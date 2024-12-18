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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  
  // Intake Motor Controllers
  private SparkMax m_lowerIntakeBar; // NEO 550 motor
  private SparkMax m_upperIntakeBar; // NEO 550 motor

  private double lowerIntakeBarRPM, upperIntakeBarRPM;

  /** Subsystem for controlling the Intake */
  public IntakeSubsystem() {
    // Instantiate the Intake motor controllers
    m_lowerIntakeBar = new SparkMax(Constants.LOWER_INTAKE_BAR_MOTOR_ID, MotorType.kBrushless);
    m_upperIntakeBar = new SparkMax(Constants.UPPER_INTAKE_BAR_MOTOR_ID, MotorType.kBrushless);

    // Configure the Spark MAX motor controllers using the new 2025 method
    configureSparkMAX(m_lowerIntakeBar, Constants.LOWER_INTAKE_BAR_INVERT);
    configureSparkMAX(m_upperIntakeBar, Constants.UPPER_INTAKE_BAR_INVERT);

    SmartDashboard.putNumber("Upper Intake Bar Speed", Constants.UPPER_INTAKE_BAR_SPEED);
    SmartDashboard.putNumber("Lower Intake Bar Speed", Constants.LOWER_INTAKE_BAR_SPEED);
  }

  private void configureSparkMAX(SparkMax max, boolean reverse) {
		SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(reverse).idleMode(IdleMode.kBrake);
    max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

  /* Set power to the intake motors */
  public void setPower(double upperPower, double lowerPower) {
    m_upperIntakeBar.set(upperPower);
    m_lowerIntakeBar.set(lowerPower);
  }
  public void stop() {
    m_lowerIntakeBar.set(0);
    m_upperIntakeBar.set(0);
  }

  /* Read the speed of the intake motors */
  public double getLowerIntakeBarRPM() {
    return lowerIntakeBarRPM;
  }
  public double getUpperIntakeBarRPM() {
    return upperIntakeBarRPM;
  }

  @Override
  public void periodic() {
    lowerIntakeBarRPM = m_lowerIntakeBar.getEncoder().getVelocity();
    upperIntakeBarRPM = m_upperIntakeBar.getEncoder().getVelocity();

    // Add intake bar RPM readingss to SmartDashboard for the sake of datalogging
    SmartDashboard.putNumber("Lower Intake Bar RPM", lowerIntakeBarRPM);
    SmartDashboard.putNumber("Upper Intake Bar RPM", upperIntakeBarRPM);
  }
}