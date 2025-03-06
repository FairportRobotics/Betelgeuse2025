package frc.robot.commands;

import frc.robot.Constants.ElevatorPositions;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorUpCommand extends ElevatorGoToLevelCommand {

    public ElevatorUpCommand(ElevatorSubsystem elevatorSubsystem) {
        super(elevatorSubsystem);
    }

    @Override
    public void execute() {
        int indexOfGoToLevel = position.ordinal();
        if (indexOfGoToLevel == ElevatorPositions.values().length - 1)
            return;
        elevatorSubsystem.moveElevator(ElevatorPositions.values()[indexOfGoToLevel + 1]);
    }
}