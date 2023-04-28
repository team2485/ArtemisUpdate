// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import pabeles.concurrency.IntOperatorTask.Min;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */

  private final TalonSRX m_talon;

  private PIDController m_positionController;

  private double minDistance, maxDistance;
  private boolean lowerBoundFound = false;
  private boolean upperBoundFound = false;
  private boolean centered = false;


  public Turret() {
      m_talon = new TalonSRX(TurretConstants.TALON_PORT);
      m_talon.configNominalOutputForward(0);
      m_talon.configNominalOutputReverse(0);
      m_talon.configPeakOutputForward(1);
      m_talon.configPeakOutputReverse(-1);
      m_talon.enableVoltageCompensation(true);
      m_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
      m_talon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
      m_talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);

      m_positionController = new PIDController(.25, .1, 0);
  }

  @Override
  public void periodic() {
    if (!upperBoundFound) {
      runTalon(.2);
      if (getForwardLimitSwitch()) {
        maxDistance = m_talon.getSelectedSensorPosition();
        upperBoundFound = true;
      }
    }
    else if (!lowerBoundFound) {
      runTalon(-.2);
      if (getReverseLimitSwitch()) {
        minDistance = m_talon.getSelectedSensorPosition();
        lowerBoundFound = true;
      }
    }
    else if (!centered) {
      runPositionalPID(0);
      if (Math.abs(0 - getEncoderDistance()) < .05) {
        runTalon(0);
        centered = true;
      }
    }
  }

  public void runPositionalPID(double position) {
    m_positionController.setSetpoint(MathUtil.clamp(position, -1, 1));
    runTalon(m_positionController.calculate(getEncoderDistance()));
  }

  public void runTalon(double power) {
    m_talon.set(ControlMode.PercentOutput, power);
  }

  public boolean getForwardLimitSwitch() {
    return !m_talon.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public boolean getReverseLimitSwitch() {
    return !m_talon.getSensorCollection().isRevLimitSwitchClosed();
  }

  public double getEncoderDistance() {
    return map(m_talon.getSelectedSensorPosition(), minDistance, maxDistance, -1, 1);
  }

  private double map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  public boolean getWasCentered() {
    return centered;
  }
}
