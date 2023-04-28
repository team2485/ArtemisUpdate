// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GeneralMagazine;
import frc.robot.subsystems.Shooter;

public class SpinFlywheel extends CommandBase {
  /** Creates a new Index. */
  
  private final Shooter m_Shooter;
  public SpinFlywheel(Shooter shoot) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Shooter = shoot;
    
    
    addRequirements(shoot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Shooter.runVelocityPID(.3, 1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Shooter.runVelocityPID(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
