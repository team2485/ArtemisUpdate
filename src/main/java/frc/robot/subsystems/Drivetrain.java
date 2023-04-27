// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants.DriveTrainConstants;

public class Drivetrain extends SubsystemBase {
  private final TalonFX leftLead = new TalonFX(DriveTrainConstants.kFrontLeftMotorPort);
  private final TalonFX leftFollower = new TalonFX(DriveTrainConstants.kBackLeftMotorPort);
  private final TalonFX rightLead = new TalonFX(DriveTrainConstants.kFrontRightMotorPort);
  private final TalonFX rightFollower = new TalonFX(DriveTrainConstants.kBackRightMotorPort);

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    rightLead.setInverted(true);
    rightFollower.setInverted(true);

    //leftFollower.follow(leftLead);
    //rightFollower.follow(rightLead);
  }

  public void drive(double xAxis, double yAxis) {
    double powerOutput = yAxis;
    double turningOutput = xAxis;
    
    double rightPowerOutput = lerp(powerOutput, 0, turningOutput);
    double leftPowerOutput = lerp(powerOutput, 0, -turningOutput);

    leftLead.set(TalonFXControlMode.PercentOutput, leftPowerOutput);
    rightLead.set(TalonFXControlMode.PercentOutput, rightPowerOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private double lerp(double a, double b, double t) {
    return a + (b - a) * t;
  }
}
