package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoIntakeCommand extends Command {
    private ArmSubsystem armSubsystem;

    /**
     * Constructs an instance of the AutoIntakeCommand.
     * @param armSubsystem is the arm subsystem the command is using.
     */
    public AutoIntakeCommand(ArmSubsystem armSubsystem)
    {
        this.armSubsystem = armSubsystem;
    }

    /**
     * Starts up the arm motor.
     */
    @Override
    public void execute()
    {
        armSubsystem.armYMotor.setNeutralMode(NeutralModeValue.Coast);
        armSubsystem.armYMotor.set(0.1);
    }

    /**
     * The command is finished when the limit switch is pressed.
     * @return true if the command is finished, false otherwise.
     */
    @Override
    public boolean isFinished()
    {
        return !armSubsystem.limitSwitch.get();
    }

    /**
     * Stops the motor
     * @param interrupted is true if the command was interrupted.
     */
    @Override
    public void end(boolean interrupted)
    {
        armSubsystem.armYMotor.set(0);
    }
}
