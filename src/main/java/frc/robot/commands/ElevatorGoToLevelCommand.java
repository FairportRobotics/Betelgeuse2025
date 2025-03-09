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
    private double requestPosRots = Double.MAX_VALUE;
    private ElevatorPositions requestedPos;

    private StatusSignal<Angle> leftPosition;
    private StatusSignal<Angle> rightPosition;

    private StatusSignal<Double> leftPosError;
    private StatusSignal<Double> rightPosError;

    final PositionVoltage rightPositionRequest;
    final PositionVoltage leftPositionRequest;

    public ElevatorGoToLevelCommand(ElevatorSubsystem elevatorSubsystem, ElevatorPositions pos) {
        _elevatorSubsystem = elevatorSubsystem;
        addRequirements(_elevatorSubsystem);

        requestPosRots = pos.getRotationUnits();
        requestedPos = pos;

        rightPosition = _elevatorSubsystem.elevatorRightMotor.getPosition();
        leftPosition = _elevatorSubsystem.elevatorLeftMotor.getPosition();

        leftPosError = _elevatorSubsystem.elevatorLeftMotor.getClosedLoopError();
        rightPosError = _elevatorSubsystem.elevatorRightMotor.getClosedLoopError();

        rightPositionRequest = new PositionVoltage(requestPosRots).withSlot(0);
        leftPositionRequest = new PositionVoltage(requestPosRots).withSlot(0);
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
        _elevatorSubsystem.elevatorLeftMotor.setNeutralMode(NeutralModeValue.Coast);
        _elevatorSubsystem.elevatorRightMotor.setNeutralMode(NeutralModeValue.Coast);

        if (_elevatorSubsystem.canGoToPosition(requestedPos)) {
            _elevatorSubsystem.elevatorLeftMotor
                    .setControl(leftPositionRequest.withPosition(_elevatorSubsystem.leftHomePos + requestPosRots));
            _elevatorSubsystem.elevatorRightMotor
                    .setControl(rightPositionRequest.withPosition(_elevatorSubsystem.rightHomePos + requestPosRots));
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {

        leftPosError.refresh();
        rightPosError.refresh();

        if (requestPosRots >= 0) {
            return _elevatorSubsystem.isAtBottom();
        } else if (leftPosition.hasUpdated() && rightPosition.hasUpdated()) {

            SmartDashboard.putNumber("Ele Left Pos", leftPosition.getValueAsDouble());
            SmartDashboard.putNumber("Ele Right", rightPosition.getValueAsDouble());

            return (Math.abs(leftPosition.getValueAsDouble() - (requestPosRots + _elevatorSubsystem.leftHomePos)) <= 0.1
                    ||
                    Math.abs(rightPosition.getValueAsDouble()
                            - (requestPosRots + _elevatorSubsystem.rightHomePos)) <= 0.1);
        }

        return false;

    }

    @Override
    public void end(boolean interrupted) {
        _elevatorSubsystem.elevatorLeftMotor.stopMotor();
        _elevatorSubsystem.elevatorRightMotor.stopMotor();
        _elevatorSubsystem.elevatorLeftMotor.setNeutralMode(NeutralModeValue.Brake);
        _elevatorSubsystem.elevatorRightMotor.setNeutralMode(NeutralModeValue.Brake);
    }

}
