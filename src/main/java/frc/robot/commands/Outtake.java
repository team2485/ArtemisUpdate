// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GeneralMagazine;
import frc.robot.subsystems.Intake;

public class Outtake extends CommandBase {
  /** Creates a new Index. */
  private final GeneralMagazine m_lowMagazine;
  private final GeneralMagazine m_highMagazine;
  private final Intake m_intake;

  public Outtake(GeneralMagazine m_lowMagazine, GeneralMagazine m_highMagazine, Intake m_intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_lowMagazine = m_lowMagazine;
    this.m_highMagazine = m_highMagazine;
    this.m_intake = m_intake;
    addRequirements(m_lowMagazine);
    addRequirements(m_highMagazine);
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_lowMagazine.runVelocityPID(-.5);
    m_highMagazine.runVelocityPID(-.5);
    m_intake.stop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_lowMagazine.runVelocityPID(0);
    m_highMagazine.runVelocityPID(0);
    m_intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
