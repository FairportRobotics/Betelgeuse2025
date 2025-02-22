// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ElevatorDownCommand;
import frc.robot.commands.ElevatorUpCommand;
import frc.robot.subsystems.HandSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  // private final HandSubsystem m_HandSubsystem = new HandSubsystem();

  // private final CommandXboxController m_driverController = new
  // CommandXboxController(
  // OperatorConstants.kDriverControllerPort);

  // public final CommandSwerveDrivetrain drivetrain =
  // TunerConstants.createDrivetrain();

  private final CommandXboxController driver = new CommandXboxController(0);
  // private final CommandXboxController opperator = new CommandXboxController(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driver.povUp().onTrue(new ElevatorUpCommand(m_ElevatorSubsystem));
    driver.povDown().onTrue(new ElevatorDownCommand(m_ElevatorSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
  // Schedule `exampleMethodCommand` when the Xbox controller's B button is
  // pressed,
  // cancelling on release.
  // m_driverController.b().onTrue(new ArmDownCommand(m_armSubsystem));

  // m_driverController.a().onTrue(new ArmUpCommand(m_armSubsystem));
}
