package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * T is some generic type that is either the position itself or a container to
 * retrieve the position.
 * PositionSubsystem compliments the logic of PositionCommand.
 */
public interface I_PositionSubsystem<T> extends Subsystem {
    /**
     * Check if the subsystem can go to the desired position.
     * 
     * @param checkPosition is the position to check if the subsystem can go to.
     * @return true if the subsystem can go to the desired position, false
     *         otherwise.
     */
    public boolean canGoToPosition(T checkPosition);

    /**
     * Set the position of the subsystem.
     * 
     * @param position is the position to set the subsystem to.
     * @return true if the position was set successfully, false otherwise.
     */
    public boolean setPosition(T setPosition);

    /**
     * Check if the subsystem is at the desired position.
     * 
     * @return true if the subsystem is at the desired position, false otherwise.
     */
    public boolean isAtPosition();

    /**
     * Stops the motors of the subsystem.
     */
    public void stopMotors();
}
