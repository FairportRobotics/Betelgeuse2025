package frc.robot.commands;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorGoToLevelCommand extends Command {

    protected ElevatorSubsystem elevatorSubsystem;
    protected double position;
    private boolean isSucessful = false;

    public ElevatorGoToLevelCommand(ElevatorSubsystem elevatorSubsystem, ElevatorPositions position) {
        Objects.requireNonNull(elevatorSubsystem, "elevator subsystem cannot be null");
        Objects.requireNonNull(position, "level cannot be null");
        addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
        this.position = position.getRotationUnits();
    }

    public ElevatorGoToLevelCommand(ElevatorSubsystem elevatorSubsystem, double position) {
        Objects.requireNonNull(elevatorSubsystem, "elevator subsystem cannot be null");
        addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
        this.position = position;
    }

    @Override
    public void execute() {
        isSucessful = elevatorSubsystem.setPosition(position);
    }

    @Override
    public boolean isFinished() {
        if (!isSucessful)
            return true;
        return elevatorSubsystem.isAtPosition();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted)
            elevatorSubsystem.stopElevator();
    }
}