package frc.robot.commands;

import frc.robot.Constants.ElevatorLevels;
import frc.robot.subsystems.ElevatorSubsystem;

public class TestElevatorUpCommand extends ElevatorGoToLevelCommand
{
    /**
     * Creates a new TestElevatorUpCommand.
     * Attempts to move the elevator up one level if possible.
     * @param elevatorSubsystem The elevator subsystem used by this command.
     */
    public TestElevatorUpCommand(ElevatorSubsystem elevatorSubsystem) {
        super(elevatorSubsystem, getLevel(elevatorSubsystem.getLevel()));
    }

    /**
     * Gets the next level of the elevator.
     * @param currentLevel is the current level of the elevator.
     * @return the next level of the elevator, returns NONE if the level of the elevator cannot be increased.
     */
    private static ElevatorLevels getLevel(ElevatorLevels currentLevel) {
        if(validToMoveUp(currentLevel))
            return ElevatorLevels.values()[currentLevel.ordinal() + 1];
        return ElevatorLevels.NONE;
    }

    /**
     * Checks if the elevator is not at the top level.
     * @param currentLevel The current level of the elevator.
     * @return true if the elevator is not at the top level, false otherwise.
     */
    private static boolean validToMoveUp(ElevatorLevels currentLevel) {
        return currentLevel != ElevatorLevels.values()[ElevatorLevels.values().length - 1];
    }
}
