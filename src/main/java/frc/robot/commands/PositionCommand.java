package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.I_PositionSubsystem;

/**
 * T is some generic type that is either the position itself or a container to
 * retrieve the position that would be used to set the position of the subsystem
 * (ex. The ElevatorSubsystem uses a double to set its position, so you
 * construct a PositionCommand<Double>(ElevatorSubsystem, position) to have the
 * ElevatorSubsystem move to that position and you can use the exact same
 * PositionCommand class for the ArmSubsystem by replacing the 'Double' generic
 * with 'ArmPositions' enum).
 * The subsystem in question must implement the I_PositionSubsystem interface
 * with the generic 'T' described above.
 * PositionCommand can be a stand-alone command or be extended for extra
 * functionality.
 */
public class PositionCommand<T> extends Command {
    protected I_PositionSubsystem<T> subsystem;
    protected T position;
    private boolean isSuccessful = false;

    public PositionCommand(I_PositionSubsystem<T> subsystem, T position) {
        this.subsystem = subsystem;
        this.position = position;
    }

    @Override
    public void execute() {
        if (subsystem.canGoToPosition(position))
            isSuccessful = subsystem.setPosition(position);
        else
            isSuccessful = false;
    }

    @Override
    public boolean isFinished() {
        if (!isSuccessful)
            return true;
        return subsystem.isAtPosition();
    }

    @Override
    public void end(boolean interrupted) {
        if (isSuccessful)
            subsystem.stopMotors();
    }
}
