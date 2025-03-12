package frc.robot.commands;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorOffCommand extends Command {

    private ElevatorSubsystem elevatorSubsystem;

    public ElevatorOffCommand(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        //elevatorSubsystem.setMotorStatus(NeutralModeValue.Brake);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
