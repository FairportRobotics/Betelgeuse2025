package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    public ElevatorSubsystem()
    {
        //Initializing electronics
    }

    /**
     * Attempts to change the level of the elevator.
     * @param increaseElevator is true if the elevator should increase in level, false if the elevator should decrease in level.
     * @return true if the elevator can change its level, false otherwise.
     */
    public boolean changeLevel(boolean increaseElevator)
    {
        //TODO unimplemented method
        throw new UnsupportedOperationException("Unimplemented method 'changeLevel'");
    }

}
