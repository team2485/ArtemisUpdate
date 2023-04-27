// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveWithControllers extends CommandBase {
  /** Creates a new DriveWithControllers. */

  private Drivetrain m_drivetrain;

  private DoubleSupplier xAxisInput;
  private DoubleSupplier yAxisInput;

  public DriveWithControllers(Drivetrain m_drivetrain, DoubleSupplier xAxisInput, DoubleSupplier yAxisInput) {
    this.m_drivetrain = m_drivetrain;
    addRequirements(m_drivetrain);

    this.xAxisInput = xAxisInput;
    this.yAxisInput = yAxisInput;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.drive(xAxisInput.getAsDouble(), yAxisInput.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
