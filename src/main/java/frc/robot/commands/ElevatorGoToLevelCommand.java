package frc.robot.commands;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorLevels;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorGoToLevelCommand extends Command {

    protected ElevatorSubsystem elevatorSubsystem;
    private ElevatorLevels level;

    public ElevatorGoToLevelCommand(ElevatorSubsystem elevatorSubsystem, ElevatorLevels level) {
        Objects.requireNonNull(elevatorSubsystem, "elevator subsystem cannot be null");
        Objects.requireNonNull(level, "level cannot be null");
        addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
        this.level = level;
    }

    public ElevatorGoToLevelCommand(ElevatorSubsystem elevatorSubsystem) {
        Objects.requireNonNull(elevatorSubsystem, "elevator subsystem cannot be null");
        addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
        this.level = elevatorSubsystem.getGoToLevel();
    }

    @Override
    public void execute() {
        elevatorSubsystem.moveElevator(level);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.isAtLevel();
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopElevator();
    }
}