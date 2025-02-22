package frc.robot.commands;

import frc.robot.Constants.ElevatorLevels;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorDownCommand extends ElevatorGoToLevelCommand {
    /**
     * Creates a new TestElevatorDownCommand.
     * Attempts to move the elevator down one level if possible.
     * 
     * @param elevatorSubsystem The elevator subsystem used by this command.
     */
    public ElevatorDownCommand(ElevatorSubsystem elevatorSubsystem) {
        super(elevatorSubsystem, getLevel(elevatorSubsystem.getGoToLevel()));
    }

    /**
     * Gets the previous level of the elevator.
     * 
     * @param currentLevel is the current level of the elevator.
     * @return the previous level of the elevator, returns NONE if the level of the
     *         elevator cannot be decreased.
     */
    private static ElevatorLevels getLevel(ElevatorLevels currentLevel) {
        if (validToMoveDown(currentLevel))
            return ElevatorLevels.values()[currentLevel.ordinal() - 1];
        return currentLevel;
    }

    /**
     * Checks if the elevator is not at the bottom level.
     * 
     * @param currentLevel The current level of the elevator.
     * @return true if the elevator is not at the bottom level, false otherwise.
     */
    private static boolean validToMoveDown(ElevatorLevels currentLevel) {
        return !ElevatorLevels.values()[0].equals(currentLevel);
    }

}