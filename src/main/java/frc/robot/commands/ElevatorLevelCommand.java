package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorLevelCommand extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    private boolean increaseLevel;

    /**
     * Constructs the ElevatorLevelCommand.
     * @param elevatorSubsystem is the elevator subsystem.
     * @param increaseLevel is true if the elevator should increase in level, false if the elevator should decrease in level.
     */
    public ElevatorLevelCommand(ElevatorSubsystem elevatorSubsystem, boolean increaseLevel)
    {
        this.elevatorSubsystem = elevatorSubsystem;
        this.increaseLevel = increaseLevel;
        addRequirements(new Subsystem[] {elevatorSubsystem});
    }

    @Override
    public void execute()
    {
        elevatorSubsystem.changeLevel(increaseLevel);
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
