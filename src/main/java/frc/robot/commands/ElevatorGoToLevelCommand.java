package frc.robot.commands;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorGoToLevelCommand extends Command {

    protected ElevatorSubsystem elevatorSubsystem;
    protected ElevatorPositions position;

    public ElevatorGoToLevelCommand(ElevatorSubsystem elevatorSubsystem, ElevatorPositions position) {
        Objects.requireNonNull(elevatorSubsystem, "elevator subsystem cannot be null");
        Objects.requireNonNull(position, "level cannot be null");
        addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
        this.position = position;
    }

    public ElevatorGoToLevelCommand(ElevatorSubsystem elevatorSubsystem) {
        Objects.requireNonNull(elevatorSubsystem, "elevator subsystem cannot be null");
        addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
        this.position = elevatorSubsystem.getGoToPosition();
    }

    @Override
    public void execute() {
        elevatorSubsystem.moveElevator(position);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.isAtPosition();
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopElevator();
    }
}