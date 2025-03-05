package frc.robot.commands;

import frc.robot.Constants.ElevatorLevels;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorDownCommand extends ElevatorGoToLevelCommand {

    public ElevatorDownCommand(ElevatorSubsystem elevatorSubsystem) {
        super(elevatorSubsystem);
    }

    @Override
    public void execute() {
        int indexOfGoToLevel = elevatorSubsystem.getGoToLevel().ordinal();
        if (indexOfGoToLevel == 0)
            return;
        elevatorSubsystem.moveElevator(ElevatorLevels.values()[indexOfGoToLevel - 1]);
    }
}