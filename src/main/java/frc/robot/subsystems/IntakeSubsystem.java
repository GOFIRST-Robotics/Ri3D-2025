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
  private SparkMax intakeBar; // NEO 550 motor

  private double intakeBarRPM;

  /** Subsystem for controlling the Intake */
  public IntakeSubsystem() {
    // Instantiate the Intake motor controllers
    intakeBar = new SparkMax(Constants.INTAKE_BAR_MOTOR_ID, MotorType.kBrushless);

    // Configure the Spark MAX motor controllers using the new 2025 method
    configureSparkMAX(intakeBar, Constants.INTAKE_BAR_INVERT);

    SmartDashboard.putNumber("Intake Bar Speed", Constants.INTAKE_BAR_SPEED);
  }

  private void configureSparkMAX(SparkMax max, boolean reverse) {
		SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(reverse).idleMode(IdleMode.kBrake);
    max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

  /* Set power to the intake motors */
  public void setPower(double power) {
    intakeBar.set(power);
  }
  public void stop() {
    intakeBar.set(0);
  }

  /* Read the speed of the intake motor */
  public double getIntakeBarRPM() {
    return intakeBarRPM;
  }

  @Override
  public void periodic() {
    intakeBarRPM = intakeBar.getEncoder().getVelocity();

    // Add intake bar RPM readingss to SmartDashboard for the sake of datalogging
    SmartDashboard.putNumber("Intake Bar RPM", intakeBarRPM);
  }
}