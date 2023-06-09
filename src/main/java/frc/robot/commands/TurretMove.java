// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Turret;

public class TurretMove extends CommandBase {
  /** Creates a new TurretMove. */
  private final Turret m_turret;
  private DoubleSupplier axis;
  public TurretMove(Turret m_turret, DoubleSupplier axis) {
    this.m_turret = m_turret;
    addRequirements(m_turret);
    // Use addRequirements() here to declare subsystem dependencies.
    this.axis = axis;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_turret.getWasCentered()) return;
    //targetPosition += xAxis.getAsDouble() * .05;
    //targetPosition = MathUtil.clamp(, -1, 1);
    m_turret.runPositionalPID(m_turret.getEncoderDistance() + MathUtil.applyDeadband(axis.getAsDouble(), OperatorConstants.kControllerDeadband));
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
