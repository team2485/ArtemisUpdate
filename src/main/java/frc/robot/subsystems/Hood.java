// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
  /** Creates a new Turret. */

  private final CANSparkMax m_spark;

  private final PIDController m_positionController;

  private double minDistance, maxDistance;
  private boolean lowerBoundFound = false;
  private boolean upperBoundFound = false;
  private boolean centered = false;


  public Hood() {
      m_spark = new CANSparkMax(HoodConstants.SPARK_PORT, MotorType.kBrushless);
      m_spark.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed).enableLimitSwitch(false);
      m_spark.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed).enableLimitSwitch(false);
      m_spark.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
      m_positionController = new PIDController(.25, .05, 0);
  }

  @Override
  public void periodic() {
    //SmartDashboard.putBoolean("Limit", getForwardLimitSwitch());
    SmartDashboard.putNumber("m_spark", m_spark.getEncoder().getPosition());
    if (!upperBoundFound) {
      runSpark(-.2);
      if (getForwardLimitSwitch()) {
        //upper bound on the hood is the lesser number
        minDistance = m_spark.getEncoder().getPosition();
        upperBoundFound = true;
      }
    }
    else if (!lowerBoundFound) {
      runSpark(.2);
      if (getReverseLimitSwitch()) {
        //lower bound on the hood is the greater number
        maxDistance = m_spark.getEncoder().getPosition();
        lowerBoundFound = true;
      }
    }
    else if (!centered) {
      runPositionalPID(0);
      if (Math.abs(0 - getEncoderDistance()) < .05) {
        runSpark(0);
        centered = true;
      }
    }
  }

  public void runPositionalPID(double position) {
    m_positionController.setSetpoint(MathUtil.clamp(position, -1, 1));
    if (Math.abs(m_positionController.getSetpoint() - getEncoderDistance()) > .01)
      runSpark(m_positionController.calculate(getEncoderDistance()));
    else 
      runSpark(0);
  }

  public void runSpark(double power) {
    m_spark.set(power);
  }

  public boolean getForwardLimitSwitch() {
    return m_spark.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed).isPressed();
  }

  public boolean getReverseLimitSwitch() {
    return m_spark.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed).isPressed();
  }

  public double getEncoderDistance() {
    return map(m_spark.getEncoder().getPosition(), minDistance, maxDistance, -1, 1);
  }

  private double map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  public boolean getWasCentered() {
    return centered;
  }
}
