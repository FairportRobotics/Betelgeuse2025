package frc.robot.commands;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorLevels;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorGoToLevelCommand extends Command {

    private ElevatorSubsystem elevatorSubsystem;
    private ElevatorLevels elevatorLevel;
    private double requestPosRots = Double.MAX_VALUE;

    private StatusSignal<Angle> leftPosition;
    private StatusSignal<Angle> rightPosition;

    private StatusSignal<Double> leftPosError;
    private StatusSignal<Double> rightPosError;

    final PositionVoltage rightPositionRequest;
    final PositionVoltage leftPositionRequest;

    public ElevatorGoToLevelCommand(ElevatorSubsystem elevatorSubsystem, ElevatorLevels elevatorLevel) {
        this.elevatorLevel = elevatorLevel;
        rightPositionRequest = new PositionVoltage(0).withSlot(0);
        leftPositionRequest = new PositionVoltage(0).withSlot(0);

        if(ElevatorLevels.NONE.equals(elevatorLevel))
            return;

        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);

        requestPosRots = getEncoderValueForLevel(elevatorLevel);

        rightPosition = elevatorSubsystem.getRightMotor().getPosition();
        leftPosition = elevatorSubsystem.getLeftMotor().getPosition();

        leftPosError = elevatorSubsystem.getLeftMotor().getClosedLoopError();
        rightPosError = elevatorSubsystem.getRightMotor().getClosedLoopError();
    }


    @Override
    public void initialize() {
        if(ElevatorLevels.NONE.equals(elevatorLevel))
            return;

        setMotorNeutralMode(NeutralModeValue.Coast);

        //Moves both motors toward
        elevatorSubsystem.getLeftMotor()
                .setControl(leftPositionRequest.withPosition(elevatorSubsystem.getLeftHomePos() + requestPosRots));
        elevatorSubsystem.getRightMotor()
                .setControl(rightPositionRequest.withPosition(elevatorSubsystem.getRightHomePos() + requestPosRots));
    }


    @Override
    public boolean isFinished() {
        if(ElevatorLevels.NONE.equals(elevatorLevel))
            return true;
        ErrorRefresh();
        if (requestPosRots <= 0) {
            return !elevatorSubsystem.getBottomLimitSwitchAsBoolean();
        } else if (leftPosition.hasUpdated() && rightPosition.hasUpdated()) {

            SmartDashboard.putNumber("Ele Left Pos", leftPosition.getValueAsDouble());
            SmartDashboard.putNumber("Ele Right", rightPosition.getValueAsDouble());

            return (Math.abs(leftPosition.getValueAsDouble() - (requestPosRots + elevatorSubsystem.getLeftHomePos())) <= 0.1
                    ||
                    Math.abs(rightPosition.getValueAsDouble()
                            - (requestPosRots + elevatorSubsystem.getRightHomePos())) <= 0.1);
        }
        return false;
    }

    /**
     * Stops the elevator motors when the command ends.
     */
    @Override
    public void end(boolean interrupted) {
        // RobotContainer.noteAquired = false;
        stopMotors();
        setMotorNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * Gets the encoder value from Constants.java for the given level.
     * @param level The level to get the encoder value for.
     * @return The encoder value for the given level.
     */
    public static double getEncoderValueForLevel(ElevatorLevels level) {
        switch (level) {
            case HOME:
                return Constants.ElevatorEncoderValues.HOME;
            case CORAL:
                return Constants.ElevatorEncoderValues.CORAL;
            case ONE:
                return Constants.ElevatorEncoderValues.ONE;
            case TWO:
                return Constants.ElevatorEncoderValues.TWO;
            case THREE:
                return Constants.ElevatorEncoderValues.THREE;
            case FOUR:
                return Constants.ElevatorEncoderValues.FOUR;
            case MAX:
                return Constants.ElevatorEncoderValues.MAX;
            default:
                throw new IllegalArgumentException("Unknown level: " + level);
        }
    }

    /**
     * Stops the elevator motors.
     */
    private void stopMotors() {
        elevatorSubsystem.getLeftMotor().stopMotor();
        elevatorSubsystem.getRightMotor().stopMotor();
    }

    /**
     * Sets the neutral mode of the elevator motors to brake.
     */
    private void setMotorNeutralMode(NeutralModeValue modeValue) {
        elevatorSubsystem.getLeftMotor().setNeutralMode(modeValue);
        elevatorSubsystem.getRightMotor().setNeutralMode(modeValue);
    }

    /**
     * Refreshes the error signals for the left and right motors.
     */
    private void ErrorRefresh() {
        leftPosError.refresh();
        rightPosError.refresh();
    }
}
