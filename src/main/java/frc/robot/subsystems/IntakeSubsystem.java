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
  private SparkMax m_DeployIntake; // NEO 550 motor

  private double IntakeBarRPM;
  private double DeployPosition;
  private double intakeMotorCurrent;
  private double intakeMotorCurrentMean;
  private double intakeMotorCurrents[];
  private int i;
  private double intakeMotorCurrentDeviationTotal;
  private double intakeMotorCurrentStandardDeviation;

  private double intakeLiftMotorCurrent;

  private double intakeGravityControl;

  private int preset;

  /** Subsystem for controlling the Intake */
  public IntakeSubsystem() {
    // Instantiate the Intake motor controllers
    m_IntakeBar = new SparkMax(Constants.INTAKE_BAR_MOTOR_ID, MotorType.kBrushless);
    m_DeployIntake = new SparkMax(Constants.DEPLOY_INTAKE_MOTOR_ID, MotorType.kBrushless);
    // Configure the Spark MAX motor controllers using the new 2025 method
    configureSparkMAX(m_IntakeBar, Constants.INTAKE_BAR_INVERT);
    configureSparkMAX(m_DeployIntake, Constants.INTAKE_DEPLOY_INVERT);
    preset = 0;
    SmartDashboard.putNumber("Intake Bar Speed", Constants.INTAKE_BAR_SPEED);
    intakeMotorCurrents = new double[10];
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
  public void setPreset(int preset) {
    this.preset = preset;
  }

  public double getIntakeGravityControl() {
    return intakeGravityControl;
  }

  public double getIntakeMotorCurrentMean() {
    return intakeMotorCurrentMean;
  }

  /* get whether the intake is in or out */
  public int getPreset() {
    return preset;
  }
  /* get position */
  public double getPosition() {
    return DeployPosition;
  }

  public double getIntakeMotorCurrent() {
    return intakeMotorCurrent;
  }

  public double getIntakeLiftMotorCurrent() {
    return intakeLiftMotorCurrent;
  }

  public double getIntakeMotorCurrentStandardDeviation() {
    intakeMotorCurrentMean = 0;
    for(int index = 0; index < 10; index++)  {
      intakeMotorCurrentMean += intakeMotorCurrents[i];
    }
    intakeMotorCurrentDeviationTotal = 0;
    for(int index = 0; index < 10; index++)  {
      intakeMotorCurrents[i] -= intakeMotorCurrentMean;
      intakeMotorCurrentDeviationTotal += Math.pow(intakeMotorCurrents[i],2);
    }
    intakeMotorCurrentStandardDeviation = Math.pow(intakeMotorCurrentDeviationTotal,0.5);
    return intakeMotorCurrentStandardDeviation;
  }

  public void deployIntake(double power) {
    if(!(power > 0 && getPosition() > Constants.INTAKE_DEPLOY_LIMIT || (power < 0 && getPosition() < Constants.INTAKE_RETURN_LIMIT))) {
      m_DeployIntake.set(power);
    } else {
      m_DeployIntake.set(0);
    }
  }

  @Override
  public void periodic() {
    IntakeBarRPM = m_IntakeBar.getEncoder().getVelocity();
    DeployPosition = m_DeployIntake.getEncoder().getPosition();
    intakeMotorCurrent = m_IntakeBar.getOutputCurrent();
    intakeLiftMotorCurrent = m_DeployIntake.getOutputCurrent();
    intakeGravityControl = DeployPosition*Constants.GRAVITY_RESISTANCE/Constants.INTAKE_DEPLOY_LIMIT;
    i = i++%10;
    intakeMotorCurrents[i] = intakeMotorCurrent;

    // Add intake bar RPM and deploy position readings to SmartDashboard for the sake of data logging
    SmartDashboard.putNumber("Intake Bar RPM", IntakeBarRPM);
    SmartDashboard.putNumber("Intake Deploy Position", DeployPosition);
  }
}