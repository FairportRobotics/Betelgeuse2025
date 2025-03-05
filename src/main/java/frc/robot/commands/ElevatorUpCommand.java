package frc.robot.commands;

import frc.robot.Constants.ElevatorLevels;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorUpCommand extends ElevatorGoToLevelCommand {

    public ElevatorUpCommand(ElevatorSubsystem elevatorSubsystem) {
        super(elevatorSubsystem);
    }

    @Override
    public void execute() {
        int indexOfGoToLevel = elevatorSubsystem.getGoToLevel().ordinal();
        if (indexOfGoToLevel == ElevatorLevels.values().length - 1)
            return;
        elevatorSubsystem.moveElevator(ElevatorLevels.values()[indexOfGoToLevel + 1]);
    }
}