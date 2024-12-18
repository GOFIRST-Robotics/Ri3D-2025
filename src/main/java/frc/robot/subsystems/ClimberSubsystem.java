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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  
  // Climber Motor Controllers
  private SparkMax m_climber; // NEO motor

  /** Subsystem for controlling the climber */
  public ClimberSubsystem() {
    // Configure the Spark MAX motor controller using the new 2025 method
    m_climber = new SparkMax(Constants.CLIMBER_MOTOR_ID_SPARK, MotorType.kBrushed);
    configureSparkMAX(m_climber, Constants.CLIMBER_INVERT);

    // Put the default speed on SmartDashboard if needed
    // SmartDashboard.putNumber("Climber Speed", Constants.CLIMBER_SPEED);
  }

  private void configureSparkMAX(SparkMax max, boolean reverse) {
		SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(reverse).idleMode(IdleMode.kBrake);
    max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

  /* Set power to the climber motor */
  public void setPower(double power) {
    // Spark Max set method
    m_climber.set(power);
  }

  public void stop() {
    setPower(0);
  }

  @Override
  public void periodic() {}
}