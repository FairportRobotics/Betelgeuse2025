package frc.robot.commands;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorAutoHomeCommand extends Command{

    private ElevatorSubsystem _ElevatorSubsystem;
    
    /**
     * Creates a new ElevatorAutoHomeCommand.
     * @param ElevatorSubsystem is the elevator subsystem used by this command.
     */
    public ElevatorAutoHomeCommand(ElevatorSubsystem ElevatorSubsystem){
        _ElevatorSubsystem = ElevatorSubsystem;
        addRequirements(_ElevatorSubsystem);
    }

    /**
     * Starts by moving the elevator down.
     */
    @Override
    public void initialize() {
        _ElevatorSubsystem.set(-0.1);
    }

    /**
     * Command is finished and its end method is run when the bottom limit switch is pressed.
     */
    @Override
    public boolean isFinished() {
        return !_ElevatorSubsystem.getBottomLimitSwitchAsBoolean();
    }

    /**
     * Stops the elevator motors and stores the encoders and home positions of the motors.
     */
    @Override
    public void end(boolean interrupted) {
        _ElevatorSubsystem.set(0.0);

        StatusSignal<Angle> leftPos = _ElevatorSubsystem.getLeftMotor().getPosition();
        StatusSignal<Angle> rightPos = _ElevatorSubsystem.getRightMotor().getPosition();

        leftPos.waitForUpdate(1.0);
        rightPos.waitForUpdate(1.0);

        _ElevatorSubsystem.setLeftHomePos(leftPos.getValueAsDouble());
        _ElevatorSubsystem.setRightHomePos(rightPos.getValueAsDouble());
    }

}
