package frc.robot.commands;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class DefaultArmDownMoveElevatorToPlayerStation extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    private ArmSubsystem armSubsystem;

    public DefaultArmDownMoveElevatorToPlayerStation(ElevatorSubsystem elevatorSubsystem,
            ArmSubsystem armSubsystem) {
        this.elevatorSubsystem = Objects.requireNonNull(elevatorSubsystem, "elevator subsystem cannot be null");
        this.armSubsystem = Objects.requireNonNull(armSubsystem, "arm subsystem cannot be null");
    }

    @Override
    public void execute() {
        if (armSubsystem.getActualPos().refresh().getValueAsDouble() < ArmPositions.MIDDLE.getValue())
            elevatorSubsystem.setPosition(ElevatorPositions.HUMAN_PLAYER_STATION.getRotationUnits());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
