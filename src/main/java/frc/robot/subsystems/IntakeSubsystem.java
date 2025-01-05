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
  private SparkMax m_IntakeBar; // NEO 550 motor
  private SparkMax m_DeployIntake; // NEO 550 motor

  private double IntakeBarRPM;
  private double DeployPosition;

  private boolean deployed;

  /** Subsystem for controlling the Intake */
  public IntakeSubsystem() {
    // Instantiate the Intake motor controllers
    m_IntakeBar = new SparkMax(Constants.INTAKE_BAR_MOTOR_ID, MotorType.kBrushless);
    m_DeployIntake = new SparkMax(Constants.DEPLOY_INTAKE_MOTOR_ID, MotorType.kBrushless);
    // Configure the Spark MAX motor controllers using the new 2025 method
    configureSparkMAX(m_IntakeBar, Constants.INTAKE_BAR_INVERT);

    SmartDashboard.putNumber("Intake Bar Speed", Constants.INTAKE_BAR_SPEED);
  }

  private void configureSparkMAX(SparkMax max, boolean reverse) {
		SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(reverse).idleMode(IdleMode.kBrake);
    max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

  /* Set power to the intake motor */
  public void setPower(double Power) {
    m_IntakeBar.set(Power);
  }
  public void stop() {
    m_IntakeBar.set(0);
  }

  /* Read the speed of the intake motor */
  public double getIntakeBarRPM() {
    return IntakeBarRPM;
  }
  /* toggle the intake in or out */
  public void deployToggle() {
    deployed = !deployed;
  }

  /* get whether the intake is in or out */
  public boolean getDeployed() {
    return deployed;
  }
  /* get position */
  public double getPosition() {
    return DeployPosition;
  }

  public void goToPosition() {
    if(deployed) {
      m_DeployIntake.getEncoder().setPosition(Constants.INTAKE_DEPLOYED_POS);
    } else {
      m_DeployIntake.getEncoder().setPosition(Constants.INTAKE_RETURNED_POS);
    }
  } 

  @Override
  public void periodic() {
    IntakeBarRPM = m_IntakeBar.getEncoder().getVelocity();
    DeployPosition = m_DeployIntake.getEncoder().getPosition();

    // Add intake bar RPM readingss to SmartDashboard for the sake of datalogging
    SmartDashboard.putNumber("Intake Bar RPM", IntakeBarRPM);
  }
}