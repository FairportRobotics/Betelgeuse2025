// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ElevatorLevels;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ArmConstants.ArmPositions;
import frc.robot.commands.ArmGotoCommand;
import frc.robot.commands.ElevatorDownCommand;
import frc.robot.commands.ElevatorGoToLevelCommand;
import frc.robot.commands.ElevatorUpCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.HandCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OutakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.HopperSubsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  // // kSpeedAt12Volts desired top speed

  // private final Telemetry logger = new Telemetry(MaxSpeed);
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem(m_armSubsystem);
  private final ClimbingSubsystem m_ClimbingSubsystem = new ClimbingSubsystem();
  private final HandSubsystem m_HandSubsystem = new HandSubsystem();
  private final HopperSubsystem m_HopperSubsystem = new HopperSubsystem(
      Commands.sequence(
          new ArmGotoCommand(m_armSubsystem, ArmPositions.DOWN),
          Commands.parallel(
              new ElevatorGoToLevelCommand(m_ElevatorSubsystem, ElevatorLevels.ONE),
              new IntakeCommand(m_HandSubsystem)),
          Commands.parallel(
              new ElevatorGoToLevelCommand(m_ElevatorSubsystem, ElevatorLevels.TWO),
              new ArmGotoCommand(m_armSubsystem, ArmPositions.MIDDLE))));

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    NamedCommands.registerCommand("Shoot", new OutakeCommand(m_HandSubsystem));
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative
                                                                                         // Y (forward)
            .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain.applyRequest(() ->
    // point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
    // -joystick.getLeftX()))
    // ));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    driver.rightTrigger().onTrue(Commands.deadline(new WaitCommand(.25), new HandCommand(m_HandSubsystem, 0)));
    // drivetrain.registerTelemetry(logger::telemeterize);

    // Test commands for testing :)
    // driver.a().onTrue();

    driver.povUp().onTrue(new ElevatorUpCommand(m_ElevatorSubsystem));
    driver.povDown().onTrue(new ElevatorDownCommand(m_ElevatorSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
  // Schedule `exampleMethodCommand` when the Xbox controller's B button is
  // pressed,
  // cancelling on release.
  // driver.b().onTrue(new ArmDownCommand(m_armSubsystem));

  // driver.a().onTrue(new ArmUpCommand(m_armSubsystem));
}
