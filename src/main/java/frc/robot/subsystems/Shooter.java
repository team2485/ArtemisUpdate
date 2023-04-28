// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax m_left;
  private CANSparkMax m_right;
  private CANSparkMax m_feeder;
  private PIDController m_leftController;
  private PIDController m_rightController;
  private PIDController m_feedeController;
  public Shooter(int leftPort, int rightPort,int feederPort) {

    m_left=new CANSparkMax(leftPort, MotorType.kBrushless);
    m_right=new CANSparkMax(rightPort, MotorType.kBrushless);
    m_feeder = new CANSparkMax(feederPort, MotorType.kBrushless);
    m_right.setInverted(true);
    m_leftController = new PIDController(1, 0, 0);
    m_rightController = new PIDController(1, 0, 0);
    m_feedeController = new PIDController(1, 0, 0);

    m_left.getEncoder().setVelocityConversionFactor(Constants.Flywheels.GEAR_RATIO);
    m_right.getEncoder().setVelocityConversionFactor(Constants.Flywheels.GEAR_RATIO);
    m_feeder.getEncoder().setVelocityConversionFactor(Constants.Feeder.GEAR_RATIO);
    
    m_left.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_right.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_feeder.setIdleMode(CANSparkMax.IdleMode.kCoast);

    m_right.setSmartCurrentLimit(Constants.Flywheels.SPARK_FLYWHEEL_LEFT_MAX_CURRENT);
    m_left.setSmartCurrentLimit(Constants.Flywheels.SPARK_FLYWHEEL_RIGHT_MAX_CURRENT);
    m_feeder.setSmartCurrentLimit(Constants.Feeder.MAX_CURRENT);

    m_left.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
    m_right.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
    m_feeder.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);

    m_leftController.setTolerance(Constants.Flywheels.RPM_THRESHOLD);
    m_rightController.setTolerance(Constants.Flywheels.RPM_THRESHOLD);
    //m_feedeController.setTolerance(Constants.Feeder.);
  }
  private void setLeftPWM(double pwm) {
    m_left.set(pwm);
}

  private void setRightPWM(double pwm) {
      m_right.set(pwm);
  }

  public void setPWM(double leftPWM, double rightPWM) {
    setLeftPWM(leftPWM);
    setRightPWM(rightPWM);
}

public void setPWM(double pwm) {
  setPWM(pwm, pwm);
}
public void resetPIDs() {
  m_leftController.reset();
  m_rightController.reset();
}

private void setLeftVelocity(double velocity) {
  m_leftController.calculate(MathUtil.clamp(velocity, Constants.Flywheels.FLYWHEELS_MIN_VELOCITY, Constants.Flywheels.FLYWHEELS_MAX_VELOCITY));
}

private void setRightVelocity(double velocity) {
  m_rightController.calculate(MathUtil.clamp(velocity, Constants.Flywheels.FLYWHEELS_MIN_VELOCITY, Constants.Flywheels.FLYWHEELS_MAX_VELOCITY));
}
public void setVelocity(double leftVelocity, double rightVelocity) {
  setLeftVelocity(leftVelocity);
  setRightVelocity(rightVelocity);
}
public void setVelocity(double velocity) {

  setVelocity(velocity, velocity);
}
public boolean atVelocitySetpoint() {
  return m_leftController.atSetpoint() && m_rightController.atSetpoint();
}

public double getLeftEncoderVelocity() {
  return m_left.getEncoder().getVelocity();
}

public double getRightEncoderVelocity() {
  return m_right.getEncoder().getVelocity();
}
public void runVelocityPID(double velocity,double feederVel) {
  //m_spark.runPID(MathUtil.clamp(velocity, Constants.Magazine.MAGAZINE_MIN_VELOCITY, Constants.Magazine.MAGAZINE_MAX_VELOCITY));
  m_left.set(m_leftController.calculate(velocity));
  m_right.set(m_rightController.calculate(velocity));
  m_feeder.set(m_feedeController.calculate(feederVel));
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
