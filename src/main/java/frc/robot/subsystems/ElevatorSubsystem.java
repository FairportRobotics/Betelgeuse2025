package frc.robot.subsystems;

import org.fairportrobotics.frc.posty.TestableSubsystem;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.ElevatorPositions;

public class ElevatorSubsystem extends TestableSubsystem {

    public double rightHomePos = Double.MAX_VALUE;
    public double leftHomePos = Double.MAX_VALUE;

    public TalonFX elevatorLeftMotor = new TalonFX(Constants.CanBusIds.ELEVATOR_LEFT_MOTOR_ID);
    public TalonFX elevatorRightMotor = new TalonFX(Constants.CanBusIds.ELEVATOR_RIGHT_MOTOR_ID);
    public DigitalInput bottomlimitSwitch;

    private StatusSignal<Angle> leftPos;
    private StatusSignal<Angle> rightPos;

    private ArmSubsystem armSubsystem;

    private double lowestValidElevatorPosition = ElevatorPositions.HOME.getRotationUnits();

    public ElevatorSubsystem(ArmSubsystem armSubsystem) {
        super("ElevatorSubsystem");

        this.armSubsystem = armSubsystem;

        this.armSubsystem.setElevatorSubsystem(this);

        // toplimitSwitch = new DigitalInput(8);
        bottomlimitSwitch = new DigitalInput(Constants.DIOValues.ELEVATOR_LIMIT_SWITCH);

        TalonFXConfiguration elevatorMotor1Config = new TalonFXConfiguration();
        elevatorMotor1Config.Slot0.kP = 0.7;
        elevatorMotor1Config.Slot0.kI = 0.5;
        elevatorMotor1Config.Slot0.kD = 0.1;
        elevatorMotor1Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        elevatorLeftMotor.getConfigurator().apply(elevatorMotor1Config);
        leftPos = elevatorLeftMotor.getPosition();
        leftPos.setUpdateFrequency(50);
        elevatorLeftMotor.optimizeBusUtilization();
        // elevatorLeftMotor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration elevatorMotor2Config = new TalonFXConfiguration();
        elevatorMotor2Config.Slot0.kP = 0.7;
        elevatorMotor2Config.Slot0.kI = 0.5;
        elevatorMotor2Config.Slot0.kD = 0.1;
        elevatorMotor2Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        elevatorRightMotor.getConfigurator().apply(elevatorMotor2Config);
        rightPos = elevatorRightMotor.getPosition();
        rightPos.setUpdateFrequency(50);
        elevatorRightMotor.optimizeBusUtilization();
        // elevatorRightMotor.setNeutralMode(NeutralModeValue.Brake);

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

                StatusSignal<Angle> leftPos = elevatorLeftMotor.getPosition();
                StatusSignal<Angle> rightPos = elevatorRightMotor.getPosition();

                leftPos.waitForUpdate(1.0);
                rightPos.waitForUpdate(1.0);

                leftHomePos = leftPos.getValueAsDouble();
                rightHomePos = rightPos.getValueAsDouble();

                this.elevatorLeftMotor.setNeutralMode(NeutralModeValue.Brake);
                this.elevatorRightMotor.setNeutralMode(NeutralModeValue.Brake);
            }

            if (armSubsystem.getActualPos().getValueAsDouble() > ArmPositions.MIDDLE.getValue()) {
                lowestValidElevatorPosition = ElevatorPositions.ARM_LIMIT.getRotationUnits();
            }
        }

        Logger.recordOutput("Elevator At Bottom", !bottomlimitSwitch.get());

        Logger.recordOutput("Elevator Left Pos", leftPos.refresh().getValue());
        Logger.recordOutput("Elevator Right Pos", rightPos.refresh().getValue());
    }

    public boolean canGoToPosition(ElevatorPositions requestedPos) {
        if (requestedPos.getRotationUnits() > lowestValidElevatorPosition)
            return true;
        else
            return false;
    }
}
