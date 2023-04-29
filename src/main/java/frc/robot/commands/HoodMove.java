// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Hood;

public class HoodMove extends CommandBase {
  /** Creates a new TurretMove. */
  private final Hood m_hood;
  private DoubleSupplier axis;
  public HoodMove(Hood m_hood, DoubleSupplier axis) {
    this.m_hood = m_hood;
    addRequirements(m_hood);
    // Use addRequirements() here to declare subsystem dependencies.
    this.axis = axis;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_hood.getWasCentered()) return;
    //targetPosition += xAxis.getAsDouble() * .05;
    //targetPosition = MathUtil.clamp(, -1, 1);
    m_hood.runPositionalPID(m_hood.getEncoderDistance() + MathUtil.applyDeadband(axis.getAsDouble(), OperatorConstants.kControllerDeadband));
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
