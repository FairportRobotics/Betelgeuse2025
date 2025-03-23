package frc.robot.commands;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorGoToLevelCommand extends Command {

    private ElevatorSubsystem _elevatorSubsystem;
    private ElevatorPositions requestedPos;

    public ElevatorGoToLevelCommand(ElevatorSubsystem elevatorSubsystem, ElevatorPositions pos) {
        _elevatorSubsystem = elevatorSubsystem;
        addRequirements(_elevatorSubsystem);

        requestedPos = pos;
    }

    // public ElevatorGoToLevelCommand(ElevatorSubsystem elevatorSubsystem, double
    // pos) {
    // _elevatorSubsystem = elevatorSubsystem;
    // addRequirements(_elevatorSubsystem);
    // requestPosRots = pos;

    // rightPosition = _elevatorSubsystem.elevatorRightMotor.getPosition();
    // leftPosition = _elevatorSubsystem.elevatorLeftMotor.getPosition();

    // leftPosError = _elevatorSubsystem.elevatorLeftMotor.getClosedLoopError();
    // rightPosError = _elevatorSubsystem.elevatorRightMotor.getClosedLoopError();

    // rightPositionRequest = new PositionVoltage(0).withSlot(0);
    // leftPositionRequest = new PositionVoltage(0).withSlot(0);
    // }

    @Override
    public void initialize() {
        _elevatorSubsystem.goToPosition(requestedPos);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return _elevatorSubsystem.isAtTargetPos();
    }

    @Override
    public void end(boolean interrupted) {
        _elevatorSubsystem.elevatorLeftMotor.stopMotor();
        _elevatorSubsystem.elevatorLeftMotor.setNeutralMode(NeutralModeValue.Brake);
    }

}
