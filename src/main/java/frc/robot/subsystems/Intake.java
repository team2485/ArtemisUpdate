// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private final CANSparkMax m_frontSpark;
  private final CANSparkMax m_sideSpark;
  
  /** Creates a new Intake. */
  public Intake(int frontPort, int sidePort) {
    m_frontSpark = new CANSparkMax(frontPort, MotorType.kBrushless);
    m_sideSpark = new CANSparkMax(sidePort, MotorType.kBrushless);
    m_frontSpark.setInverted(true);
  }

  public void intake() {
    m_frontSpark.set(.4);
    m_sideSpark.set(.4);
  }

  public void outtake() {
    m_frontSpark.set(-.4);
    m_sideSpark.set(-.4);
  }

  public void stop() {
    m_frontSpark.stopMotor();
    m_sideSpark.stopMotor();
  }
}
