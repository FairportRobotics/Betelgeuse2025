package frc.robot.subsystems;

import org.fairportrobotics.frc.posty.TestableSubsystem;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.ElevatorPositions;

public class ElevatorSubsystem extends TestableSubsystem {

    private final double DEFAULT_HOME_POS = 0.00001;

    public double homePos = DEFAULT_HOME_POS;

    public static TalonFX elevatorLeftMotor = new TalonFX(Constants.CanBusIds.ELEVATOR_LEFT_MOTOR_ID);
    public static TalonFX elevatorRightMotor = new TalonFX(Constants.CanBusIds.ELEVATOR_RIGHT_MOTOR_ID);
    private DigitalInput bottomlimitSwitch;

    private StatusSignal<Angle> currentPos;

    private StatusSignal<Double> requestedPos;

    private StatusSignal<Double> pidError;

    final PositionVoltage positionRequest;

    private ArmSubsystem armSubsystem;

    private double lowestValidElevatorPosition = ElevatorPositions.HOME.getRotationUnits();

    Alert ArmBlockingAlert = new Alert("ARM is blocking Elevator movement",AlertType.kWarning);

    public ElevatorSubsystem(ArmSubsystem armSubsystem) {
        super("ElevatorSubsystem");

        this.armSubsystem = armSubsystem;

        this.armSubsystem.setElevatorSubsystem(this);

        bottomlimitSwitch = new DigitalInput(Constants.DIOValues.ELEVATOR_LIMIT_SWITCH);

        TalonFXConfiguration elevatorMotorConfig = new TalonFXConfiguration();
        elevatorMotorConfig.Slot0.kP = .8;
        elevatorMotorConfig.Slot0.kI = 0;
        elevatorMotorConfig.Slot0.kD = 0;
        elevatorMotorConfig.Feedback.RotorToSensorRatio = 1.0;
        elevatorMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        elevatorMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // elevatorMotor1Config.CurrentLimits.StatorCurrentLimit = 160;
        elevatorMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;


        positionRequest = new PositionVoltage(0).withSlot(0);

        elevatorLeftMotor.getConfigurator().apply(elevatorMotorConfig);
        elevatorLeftMotor.setControl(positionRequest);

        elevatorRightMotor.setControl(new Follower(elevatorLeftMotor.getDeviceID(), true));

        currentPos = elevatorLeftMotor.getRotorPosition();
        currentPos.setUpdateFrequency(50);

        requestedPos = elevatorLeftMotor.getClosedLoopReference();
        requestedPos.setUpdateFrequency(50);

        elevatorLeftMotor.optimizeBusUtilization();
        // elevatorLeftMotor.setNeutralMode(NeutralModeValue.Brake);

        elevatorRightMotor.optimizeBusUtilization();
        // elevatorRightMotor.setNeutralMode(NeutralModeValue.Brake);

        pidError = elevatorLeftMotor.getClosedLoopError();
        pidError.setUpdateFrequency(50);

        // registerPOSTTest("Left Motor Connected", () -> {
        //     return elevatorLeftMotor.isConnected();
        // });

        // registerPOSTTest("Right Motor Connected", () -> {
        //     return elevatorRightMotor.isConnected();
        // });

    }

    @Override
    public void periodic() {
        if (homePos == DEFAULT_HOME_POS) {

            if (isAtBottom()) {
                this.elevatorLeftMotor.set(0.0);

                StatusSignal<Angle> leftPos = elevatorLeftMotor.getPosition();

                leftPos.waitForUpdate(1.0);

                homePos = leftPos.getValueAsDouble();

                this.elevatorLeftMotor.setNeutralMode(NeutralModeValue.Brake);
                return;
            }

            this.elevatorLeftMotor.set(0.1); // TODO: CONSTANT!!!

        }

        if (armSubsystem.getActualPos().refresh().getValueAsDouble() > ArmPositions.MIDDLE.getValue()) {
            lowestValidElevatorPosition = ElevatorPositions.ARM_LIMIT.getRotationUnits();
        }

        Logger.recordOutput("Elevator At Bottom", isAtBottom());

        Logger.recordOutput("Elevator Pos", currentPos.refresh().getValueAsDouble()-homePos);

        Logger.recordOutput("Elevator Requested Pos", requestedPos.refresh().getValueAsDouble() - homePos);
        Logger.recordOutput("Elevator Err", pidError.refresh().getValueAsDouble());

        Logger.recordOutput("Elevator Lowest valid pos", lowestValidElevatorPosition);

        Logger.recordOutput("Elevator At Target Position", isAtTargetPos());
   }

    public boolean isAtBottom(){
      return bottomlimitSwitch.get();
    }

    public void goToPosition(ElevatorPositions targetPos){
        if (canGoToPosition(targetPos)) {
            elevatorLeftMotor
                    .setControl(positionRequest.withPosition(homePos + targetPos.getRotationUnits()));
        }
    }

    public boolean isAtTargetPos(){

        Double leftErr = pidError.refresh().getValue();

        if(leftErr == 0.0) return false;
        return Math.abs(leftErr) <= 1.2;
    }

    public void setDrive(double drive){
        elevatorLeftMotor.set(drive);
        elevatorRightMotor.set(drive);
    }

    public double getActualPos(){
        return currentPos.refresh().getValueAsDouble()-homePos;
    }

    public boolean canGoToPosition(ElevatorPositions requestedPos) {
        if (armSubsystem.getArmPos().getValue() < Constants.ArmPositions.MIDDLE.getValue()) {
            ArmBlockingAlert.set(false);
            return true;
        } else {
            ArmBlockingAlert.set(true);
            return false;
        }
    }
}
