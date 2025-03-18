// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ElevatorPositions;
import frc.robot.Constants.ControllerIds;
import frc.robot.Constants.DriveWaypoints;
import frc.robot.Constants.ArmPositions;
import frc.robot.commands.ArmGotoCommand;
import frc.robot.commands.ClimberIn;
import frc.robot.commands.ClimberOut;
import frc.robot.commands.ElevatorGoToLevelCommand;
import frc.robot.commands.ElevatorUpCommand;
import frc.robot.commands.HandCommand;
import frc.robot.commands.IntakeCoralCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.HopperSubsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private final SendableChooser<Command> autoChooser;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    // // kSpeedAt12Volts desired top speed

    // private final Telemetry logger = new Telemetry(MaxSpeed);
    private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
    private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem(m_armSubsystem);
    private final ClimbingSubsystem m_ClimbingSubsystem = new ClimbingSubsystem();
    private final HandSubsystem m_HandSubsystem = new HandSubsystem();
    private final HopperSubsystem m_HopperSubsystem = new HopperSubsystem(
            Commands.sequence(
                    new ArmGotoCommand(m_armSubsystem, ArmPositions.DOWN),
                    Commands.parallel(
                            new ElevatorGoToLevelCommand(m_elevatorSubsystem, ElevatorPositions.HUMAN_PLAYER_STATION),
                            new HandCommand(m_HandSubsystem, -.1)), // TODO: might need to change the spped to a constant later
                    Commands.parallel(
                            new ElevatorGoToLevelCommand(m_elevatorSubsystem, ElevatorPositions.TWO),
                            new ArmGotoCommand(m_armSubsystem, ArmPositions.MIDDLE))));

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Replace with CommandPS4Controller or CommandJoystick if needed
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final SwerveRequest.ApplyRobotSpeeds m_robotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    private final CommandXboxController driver = new CommandXboxController(ControllerIds.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operator = new CommandXboxController(ControllerIds.OPERATOR_CONTROLLER_PORT);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */

    public RobotContainer() {
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    drivetrain::getPose, // Robot pose supplier
                    drivetrain::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                    drivetrain::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds, feedforwards) -> drivetrain.setControl(m_robotSpeeds.withSpeeds(speeds)), // Method that
                                                                                                         // will drive
                                                                                                         // the robot
                                                                                                         // given ROBOT
                                                                                                         // RELATIVE
                                                                                                         // ChassisSpeeds.
                                                                                                         // Also
                                                                                                         // optionally
                                                                                                         // outputs
                                                                                                         // individual
                                                                                                         // module
                                                                                                         // feedforwards
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller
                                                    // for holonomic drive trains
                            new PIDConstants(1.25, 0.0, 0.1), // Translation PID constants
                            new PIDConstants(1.0, 0.0, 0.25) // Rotation PID constants
                    ),
                    config, // The robot configuration
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red
                        // alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    drivetrain // Reference to this subsystem to set requirements
            );

        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        autoChooser = AutoBuilder.buildAutoChooser(); // We can set a default auto command by passing
                                                      // a string into this function.

        SmartDashboard.putData(autoChooser);

        // Register command for PathPlanner here
        NamedCommands.registerCommand("Shoot", new HandCommand(m_HandSubsystem, .1)); // TODO: might need to change the spped to a constant later


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
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * Math.abs(-driver.getLeftY()) * MaxSpeed) // Drive forward with
                                                                                                 // negative
                                                                                                 // Y (forward)
                        .withVelocityY(-driver.getLeftX() * Math.abs(-driver.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driver.getRightX() * Math.abs(-driver.getRightX()) * MaxAngularRate) // Drive counterclockwise with
                                                                                  // negative X (left)
                ));

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        // point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
        // -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driver.leftTrigger().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //driver.a().onTrue(new IntakeCoralCommand(m_armSubsystem, m_HandSubsystem, m_elevatorSubsystem, m_HopperSubsystem));
       operator.povUp().onTrue(new ElevatorGoToLevelCommand(m_elevatorSubsystem, ElevatorPositions.THREE));
       operator.povLeft().onTrue(new ElevatorGoToLevelCommand(m_elevatorSubsystem, ElevatorPositions.TWO));
       operator.povRight().onTrue(new ElevatorGoToLevelCommand(m_elevatorSubsystem, ElevatorPositions.FOUR));
       operator.povDown().onTrue(new ElevatorGoToLevelCommand(m_elevatorSubsystem, ElevatorPositions.ONE));

        //driver.x().onTrue(new ClimberOut(m_ClimbingSubsystem));
        //driver.y().onTrue(new ClimberIn(m_ClimbingSubsystem));

        operator.rightTrigger().onTrue(Commands.deadline(new WaitCommand(2), new HandCommand(m_HandSubsystem,-.3))); // Intake
        operator.leftTrigger().onTrue(Commands.deadline(new WaitCommand(1), new HandCommand(m_HandSubsystem, .2))); // Outake
        operator.rightBumper().onTrue(Commands.deadline(new WaitCommand(1), new ArmGotoCommand(m_armSubsystem, ArmPositions.DOWN)));
        operator.leftBumper().onTrue(Commands.deadline(new WaitCommand(1), new ElevatorGoToLevelCommand(m_elevatorSubsystem, ElevatorPositions.HUMAN_PLAYER_STATION)));
        operator.a().onTrue((Commands.deadline(new WaitCommand(1), new ArmGotoCommand(m_armSubsystem, ArmPositions.MIDDLE))));
        operator.b().onTrue((Commands.deadline(new WaitCommand(1), new ArmGotoCommand(m_armSubsystem, ArmPositions.SCORINGTOP))));
        operator.x().onTrue((Commands.deadline(new WaitCommand(1), new ArmGotoCommand(m_armSubsystem, ArmPositions.STOWED))));
        driver.a().onTrue((Commands.deadline(new WaitCommand(1), new ClimberOut(m_ClimbingSubsystem))));
        driver.b().onTrue((Commands.deadline(new WaitCommand(1), new ClimberIn(m_ClimbingSubsystem))));
        //driver.b().onTrue(drivetrain.driveToWaypoint(DriveWaypoints.REEF_L));
        // drivetrain.registerTelemetry(logger::telemeterize);

        // Test commands for testing :)
        // driver.a().onTrue();

        // driver.povUp().onTrue(new ElevatorUpCommand(m_ElevatorSubsystem));
        // driver.povDown().onTrue(new ElevatorDownCommand(m_ElevatorSubsystem));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
 
    }


    public Command getAutoScoreCommand(DriveWaypoints scoringSpot, ElevatorPositions scoringLevel){
        return Commands.sequence(
            Commands.parallel(
                new ArmGotoCommand(m_armSubsystem, ArmPositions.SCORINGTOP),
                new ElevatorGoToLevelCommand(m_elevatorSubsystem, scoringLevel),
                drivetrain.driveToWaypoint(scoringSpot)
            ),
            new ArmGotoCommand(m_armSubsystem, ArmPositions.SCORINGBOTTOM),
            drivetrain.driveToWaypoint(scoringSpot)
        );
    }
}
