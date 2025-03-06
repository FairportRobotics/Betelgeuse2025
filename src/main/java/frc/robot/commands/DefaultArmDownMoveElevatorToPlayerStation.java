package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class DefaultArmDownMoveElevatorToPlayerStation extends ElevatorGoToLevelCommand {
    private ArmSubsystem armSubsystem;

    public DefaultArmDownMoveElevatorToPlayerStation(ElevatorSubsystem elevatorSubsystem,
            ArmSubsystem armSubsystem) {
        super(elevatorSubsystem);
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void execute() {
        if (armSubsystem.getArmPos() == Constants.ArmPositions.DOWN)
            elevatorSubsystem.moveElevator(ElevatorPositions.HUMAN_PLAYER_STATION);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
