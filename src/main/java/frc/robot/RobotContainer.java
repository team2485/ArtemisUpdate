// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveWithControllers;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.RunIndex;
import frc.robot.commands.SpinFlywheel;
import frc.robot.commands.TurretMove;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GeneralMagazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final GeneralMagazine m_lowMagazine = new GeneralMagazine(Constants.Magazine.SPARK_LOW_PORT,Constants.Magazine.LOW_GEAR_RATIO,Constants.Magazine.SPARK_LOW_MAX_CURRENT,true);
  private final GeneralMagazine m_highMagazine = new GeneralMagazine(Constants.Magazine.SPARK_HIGH_PORT,Constants.Magazine.HIGH_GEAR_RATIO,Constants.Magazine.SPARK_HIGH_MAX_CURRENT,false);
  private final Shooter m_Shooter = new Shooter(Constants.Flywheels.SPARK_FLYWHEEL_LEFT_PORT, Constants.Flywheels.SPARK_FLYWHEEL_RIGHT_PORT,Constants.Feeder.SPARK_PORT);
  private final RunIndex runIndex = new RunIndex(m_lowMagazine,m_highMagazine);
  private final SpinFlywheel spinFly = new SpinFlywheel(m_Shooter);
  private final Turret m_turret = new Turret();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_drivetrain.setDefaultCommand(new DriveWithControllers(m_drivetrain, ()-> m_driverController.getLeftX(), ()-> m_driverController.getLeftY()));
    m_turret.setDefaultCommand(new TurretMove(m_turret, ()-> m_driverController.getRightX()));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    m_driverController.a().whileTrue(new InstantCommand(()-> runIndex.execute())).onFalse(new InstantCommand(()->runIndex.end(false)));
    m_driverController.x().whileTrue(new InstantCommand(()-> spinFly.execute())).onFalse(new InstantCommand(()->spinFly.end(false)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
