package frc.robot.subsystems;

import java.util.Objects;

import org.fairportrobotics.frc.posty.TestableSubsystem;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.commands.DefaultArmDownMoveElevatorToPlayerStation;
import frc.robot.Constants.ArmPositions;

public class ElevatorSubsystem extends TestableSubsystem {
    private ElevatorPositions goToPosition = ElevatorPositions.HOME;

    private double rightHomePos = Double.MAX_VALUE;
    private double leftHomePos = Double.MAX_VALUE;

    public TalonFX elevatorLeftMotor = new TalonFX(Constants.CanBusIds.ELEVATOR_LEFT_MOTOR_ID);
    public TalonFX elevatorRightMotor = new TalonFX(Constants.CanBusIds.ELEVATOR_RIGHT_MOTOR_ID);
    public DigitalInput bottomlimitSwitch;

    private StatusSignal<Angle> leftPos;
    private StatusSignal<Angle> rightPos;

    private StatusSignal<Double> leftError;
    private StatusSignal<Double> rightError;

    private ArmSubsystem armSubsystem;

    private double lowestValidElevatorPosition = ElevatorPositions.HOME.getRotationUnits();

    public ElevatorSubsystem() {
        super("ElevatorSubsystem");

        // toplimitSwitch = new DigitalInput(8);
        bottomlimitSwitch = new DigitalInput(Constants.DIOValues.ELEVATOR_LIMIT_SWITCH);

        TalonFXConfiguration elevatorMotor1Config = new TalonFXConfiguration();
        elevatorMotor1Config.Slot0.kP = 0.7;
        elevatorMotor1Config.Slot0.kI = 0.5;
        elevatorMotor1Config.Slot0.kD = 0.1;
        elevatorMotor1Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        elevatorLeftMotor.getConfigurator().apply(elevatorMotor1Config);
        leftPos = elevatorLeftMotor.getPosition();
        leftPos.setUpdateFrequency(0);
        leftError = elevatorLeftMotor.getClosedLoopError();
        leftError.setUpdateFrequency(0);
        elevatorLeftMotor.optimizeBusUtilization();
        elevatorLeftMotor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration elevatorMotor2Config = new TalonFXConfiguration();
        elevatorMotor2Config.Slot0.kP = 0.7;
        elevatorMotor2Config.Slot0.kI = 0.5;
        elevatorMotor2Config.Slot0.kD = 0.1;
        elevatorMotor2Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        elevatorRightMotor.getConfigurator().apply(elevatorMotor2Config);
        rightPos = elevatorRightMotor.getPosition();
        rightPos.setUpdateFrequency(0);
        rightError = elevatorRightMotor.getClosedLoopError();
        rightError.setUpdateFrequency(0);
        elevatorRightMotor.optimizeBusUtilization();
        elevatorRightMotor.setNeutralMode(NeutralModeValue.Brake);

        registerPOSTTest("Left Motor Connected", () -> {
            return elevatorLeftMotor.isConnected();
        });

        registerPOSTTest("Right Motor Connected", () -> {
            return elevatorRightMotor.isConnected();
        });
    }

    @Override
    public void periodic() {
        if (leftHomePos == Double.MAX_VALUE || rightHomePos == Double.MAX_VALUE) {

            this.elevatorLeftMotor.set(-0.1);
            this.elevatorRightMotor.set(-0.1);

            if (!this.bottomlimitSwitch.get()) {
                this.elevatorLeftMotor.set(0.0);
                this.elevatorRightMotor.set(0.0);

                leftHomePos = leftPos.refresh().getValueAsDouble();
                rightHomePos = rightPos.refresh().getValueAsDouble();
            }

        }
        if (armSubsystem.getActualPos().refresh().getValueAsDouble() > ArmPositions.MIDDLE.getValue())
            lowestValidElevatorPosition = ElevatorPositions.ARM_LIMIT.getRotationUnits();
        else
            lowestValidElevatorPosition = 0;
        Logger.recordOutput("Elevator At Bottom", !bottomlimitSwitch.get());

        Logger.recordOutput("Elevator Left Pos", leftPos.refresh().getValue());
        Logger.recordOutput("Elevator Right Pos", rightPos.refresh().getValue());
    }

    private boolean canGoToPosition(ElevatorPositions requestedPosition) {
        return requestedPosition.getRotationUnits() > lowestValidElevatorPosition;
    }

    /**
     * Check for the position of elevator.
     * 
     * @return true if the elevator is currently at the correct position or needs to
     *         stop for other reasons.
     */
    public boolean isAtPosition() {
        if (getDefaultCommand() instanceof DefaultArmDownMoveElevatorToPlayerStation
                && armSubsystem.getArmPos() == Constants.ArmPositions.DOWN)
            return true;
        if (goToPosition == ElevatorPositions.HOME)
            return bottomlimitSwitch.get();
        return Math.abs(leftError.refresh().getValueAsDouble()) < 0.1
                || Math.abs(rightError.refresh().getValueAsDouble()) < 0.1;
    }

    /**
     * Move the elevator to the desired position.
     * 
     * @param setPosition The position to move the elevator to.
     */
    public void moveElevator(ElevatorPositions setPosition) {
        if (leftHomePos == Double.MAX_VALUE || rightHomePos == Double.MAX_VALUE
                || goToPosition == Objects.requireNonNull(setPosition, "position cannot be null"))
            return;
        goToPosition = setPosition;
        if (!canGoToPosition(setPosition))
            goToPosition = ElevatorPositions.ARM_LIMIT;
        if (goToPosition == ElevatorPositions.HOME) {
            elevatorLeftMotor.set(-0.1);
            elevatorRightMotor.set(-0.1);
        } else {
            elevatorLeftMotor.setControl(new PositionVoltage(leftHomePos + goToPosition.getRotationUnits()));
            elevatorRightMotor.setControl(new PositionVoltage(rightHomePos + goToPosition.getRotationUnits()));
        }
    }

    /**
     * Move the elevator to the desired position.
     * 
     * @param position The position to move the elevator to.
     */
    public void moveElevator(double position) {
        elevatorLeftMotor.setControl(new PositionVoltage(leftHomePos + position));
        elevatorRightMotor.setControl(new PositionVoltage(rightHomePos + position));
    }

    /**
     * Stops the elevator.
     */
    public void stopElevator() {
        elevatorLeftMotor.stopMotor();
        elevatorRightMotor.stopMotor();
    }

    /**
     * Get the position the elevator is currently at.
     * 
     * @return The position the elevator is currently at.
     */
    public ElevatorPositions getGoToPosition() {
        return goToPosition;
    }

    /**
     * Set the arm subsystem.
     * 
     * @param armSubsystem is the armSubsystem to be set.
     */
    public void setArmSubsystem(ArmSubsystem armSubsystem) {
        this.armSubsystem = Objects.requireNonNull(armSubsystem, "armSubsystem cannot be null");
    }
}
