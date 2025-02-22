package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorLevels;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorGoToLevelCommand extends Command {
    @FunctionalInterface
    public interface EncoderGetter {
        double get(ElevatorLevels level);
    }

    // Protected ensures that the variable is only accessible to the class and its
    // subclasses.
    protected ElevatorSubsystem elevatorSubsystem;
    // The level to set the elevator to.
    protected ElevatorLevels goToLevel;
    // Run before the execution of this superclass' execution methods.
    protected Runnable preExecutionRunnable;
    public static final EncoderGetter ENCODER_GETTER = ElevatorGoToLevelCommand::getEncoderValueForLevel;

    /**
     * Creates a new ElevatorGoToLevelCommand.
     * 
     * @param elevatorSubsystem is the elevator subsystem used by this command.
     * @param goToLevel         is the level to set the elevator to.
     */
    public ElevatorGoToLevelCommand(ElevatorSubsystem elevatorSubsystem, ElevatorLevels goToLevel) {
        addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
        this.goToLevel = goToLevel;
    }

    /**
     * Creates a new ElevatorGoToLevelCommand, Accessible only to the class and its
     * subclasses.
     * 
     * @param elevatorSubsystem    is the elevator subsystem used by this command.
     * @param preExecutionRunnable is the code to run before the command sets the
     *                             level of the elevator.
     */
    protected ElevatorGoToLevelCommand(ElevatorSubsystem elevatorSubsystem) {
        addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
    }

    /**
     * Sets the level of the elevator to the given level after executing any
     * existing runnables.
     */
    @Override
    public void execute() {
        elevatorSubsystem.setLevel(goToLevel);
    }

    /**
     * This command is always finished after it is executed once. Cannot be
     * overridden.
     */
    @Override
    public final boolean isFinished() {
        return elevatorSubsystem.isFinishedMoving();
    }

    /**
     * Gets the encoder value from Constants.java for the given level.
     * 
     * @param level is the level to get the encoder value for.
     * @return the encoder value for the given level.
     * @throws IllegalArgumentException if the level is not recognized.
     */
    public static double getEncoderValueForLevel(ElevatorLevels level) {
        switch (level) {
            case HOME:
                return Constants.ElevatorEncoderValues.HOME;
            case CORAL:
                return Constants.ElevatorEncoderValues.CORAL;
            case ONE:
                return Constants.ElevatorEncoderValues.ONE;
            case TWO:
                return Constants.ElevatorEncoderValues.TWO;
            case THREE:
                return Constants.ElevatorEncoderValues.THREE;
            case FOUR:
                return Constants.ElevatorEncoderValues.FOUR;
            // case MAX:
            // return Constants.ElevatorEncoderValues.MAX;
            default:
                throw new IllegalArgumentException("Unknown level: " + level);
        }
    }
}
