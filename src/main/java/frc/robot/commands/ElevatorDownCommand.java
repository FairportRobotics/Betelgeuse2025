package frc.robot.commands;

// import frc.robot.Constants.ElevatorPositions;
// import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorDownCommand extends ElevatorGoToLevelCommand {

    public ElevatorDownCommand(ElevatorSubsystem elevatorSubsystem) {
        super(elevatorSubsystem);
    }

    @Override
    public void execute() {
        int indexOfGoToLevel = position.ordinal();
        if (indexOfGoToLevel == 0)
            return;
        elevatorSubsystem.moveElevator(ElevatorPositions.values()[indexOfGoToLevel - 1]);
    }
}