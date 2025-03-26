package frc.robot.subsystems;

import org.fairportrobotics.frc.posty.TestableSubsystem;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

    public double rightHomePos = DEFAULT_HOME_POS;
    public double leftHomePos = DEFAULT_HOME_POS;

    public static TalonFX elevatorLeftMotor = new TalonFX(Constants.CanBusIds.ELEVATOR_LEFT_MOTOR_ID);
    public static TalonFX elevatorRightMotor = new TalonFX(Constants.CanBusIds.ELEVATOR_RIGHT_MOTOR_ID);
    private DigitalInput bottomlimitSwitch;

    private StatusSignal<Angle> leftPos;
    private StatusSignal<Angle> rightPos;

    private StatusSignal<Double> leftRequestedPos;
    private StatusSignal<Double> rightRequestedPos;

    private StatusSignal<Double> leftError;
    private StatusSignal<Double> rightError;

    final PositionVoltage rightPositionRequest;
    final PositionVoltage leftPositionRequest;

    private ArmSubsystem armSubsystem;

    private double lowestValidElevatorPosition = ElevatorPositions.HOME.getRotationUnits();

    Alert ArmBlockingAlert = new Alert("ARM is blocking Elevator movement",AlertType.kWarning);

    ElevatorPositions targetPos = ElevatorPositions.HOME;

    public ElevatorSubsystem(ArmSubsystem armSubsystem) {
        super("ElevatorSubsystem");

        this.armSubsystem = armSubsystem;

        this.armSubsystem.setElevatorSubsystem(this);

        bottomlimitSwitch = new DigitalInput(Constants.DIOValues.ELEVATOR_LIMIT_SWITCH);

        TalonFXConfiguration elevatorMotor1Config = new TalonFXConfiguration();
        elevatorMotor1Config.Slot0.kP = .8;
        elevatorMotor1Config.Slot0.kI = 0;
        elevatorMotor1Config.Slot0.kD = 0;
        elevatorMotor1Config.Slot1.kP = 0.2;
        elevatorMotor1Config.Slot1.kI = 0;
        elevatorMotor1Config.Slot1.kD = 0;
        elevatorMotor1Config.Feedback.RotorToSensorRatio = 1.0;
        elevatorMotor1Config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        elevatorMotor1Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // elevatorMotor1Config.CurrentLimits.StatorCurrentLimit = 160;
        elevatorMotor1Config.CurrentLimits.StatorCurrentLimitEnable = true;

        elevatorLeftMotor.getConfigurator().apply(elevatorMotor1Config);
        leftPos = elevatorLeftMotor.getRotorPosition();
        leftPos.setUpdateFrequency(50);

        leftRequestedPos = elevatorLeftMotor.getClosedLoopReference();
        leftRequestedPos.setUpdateFrequency(50);

        elevatorLeftMotor.optimizeBusUtilization();
        // elevatorLeftMotor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration elevatorMotor2Config = new TalonFXConfiguration();
        elevatorMotor2Config.Slot0.kP = .8;
        elevatorMotor2Config.Slot0.kI = 0;
        elevatorMotor2Config.Slot0.kD = 0;
        elevatorMotor2Config.Slot1.kP = 0.2;
        elevatorMotor2Config.Slot1.kI = 0.0;
        elevatorMotor2Config.Slot1.kD = 0.0;
        elevatorMotor2Config.Feedback.RotorToSensorRatio = 1.0;
        elevatorMotor2Config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        elevatorMotor2Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // elevatorMotor2Config.CurrentLimits.StatorCurrentLimit = 160;
        elevatorMotor2Config.CurrentLimits.StatorCurrentLimitEnable = true;

        elevatorRightMotor.getConfigurator().apply(elevatorMotor2Config);
        rightPos = elevatorRightMotor.getRotorPosition();
        rightPos.setUpdateFrequency(50);

        rightRequestedPos = elevatorRightMotor.getClosedLoopReference();
        rightRequestedPos.setUpdateFrequency(50);

        elevatorRightMotor.optimizeBusUtilization();
        // elevatorRightMotor.setNeutralMode(NeutralModeValue.Brake);

        leftError = elevatorLeftMotor.getClosedLoopError();
        leftError.setUpdateFrequency(50);

        rightError = elevatorRightMotor.getClosedLoopError();
        rightError.setUpdateFrequency(50);

        rightPositionRequest = new PositionVoltage(0);
        leftPositionRequest = new PositionVoltage(0);

        // registerPOSTTest("Left Motor Connected", () -> {
        //     return elevatorLeftMotor.isConnected();
        // });

        // registerPOSTTest("Right Motor Connected", () -> {
        //     return elevatorRightMotor.isConnected();
        // });

    }

    @Override
    public void periodic() {
        //if (leftHomePos == DEFAULT_HOME_POS || rightHomePos == DEFAULT_HOME_POS) {
        //
        //    if (isAtBottom()) {
        //        this.elevatorLeftMotor.set(0.0);
        //        this.elevatorRightMotor.set(0.0);
        //
        //        StatusSignal<Angle> leftPos = elevatorLeftMotor.getPosition();
        //        StatusSignal<Angle> rightPos = elevatorRightMotor.getPosition();
        //
        //        leftPos.waitForUpdate(1.0);
        //        rightPos.waitForUpdate(1.0);
        //
        //        leftHomePos = leftPos.getValueAsDouble();
        //        rightHomePos = rightPos.getValueAsDouble();
        //
        //        this.elevatorLeftMotor.setNeutralMode(NeutralModeValue.Brake);
        //        this.elevatorRightMotor.setNeutralMode(NeutralModeValue.Brake);
        //        return;
        //    }
        //
        //    this.elevatorLeftMotor.set(0.1); // TODO: CONSTANT!!!
        //    this.elevatorRightMotor.set(0.1); // TODO: CONSTANT!!!
        //
        //}

        if (armSubsystem.getActualPos().getValueAsDouble() > ArmPositions.MIDDLE.getValue()) {
            lowestValidElevatorPosition = ElevatorPositions.ARM_LIMIT.getRotationUnits();
        }

        Logger.recordOutput("Elevator At Bottom", isAtBottom());

        Logger.recordOutput("Elevator Pos", leftPos.refresh().getValueAsDouble()-leftHomePos);

        Logger.recordOutput("Elevator Requested Pos", leftRequestedPos.refresh().getValueAsDouble() - leftHomePos);
        Logger.recordOutput("Elevator Error", getPosError());

        Logger.recordOutput("Elevator Lowest valid pos", lowestValidElevatorPosition);

        Logger.recordOutput("Elevator At Target Position", isAtTargetPos());
   }

    public boolean isAtBottom(){
      return bottomlimitSwitch.get();
    }

    public void goToPosition(ElevatorPositions targetPos){
        this.targetPos = targetPos;
        //if (canGoToPosition(targetPos)) {

        if(targetPos.getRotationUnits() <= getActualPos() ){ 
            elevatorLeftMotor
                    .setControl(leftPositionRequest.withSlot(0).withPosition(leftHomePos + targetPos.getRotationUnits()));
            elevatorRightMotor
                    .setControl(rightPositionRequest.withSlot(0).withPosition(rightHomePos + targetPos.getRotationUnits()));
        }else{
            elevatorLeftMotor
                    .setControl(leftPositionRequest.withSlot(1).withPosition(leftHomePos + targetPos.getRotationUnits()));
            elevatorRightMotor
                    .setControl(rightPositionRequest.withSlot(1).withPosition(rightHomePos + targetPos.getRotationUnits()));
        }

        //}
    }

    public double getPosError(){
      double leftError = getActualPos() - this.targetPos.getRotationUnits();
      return leftError;
    }

    public boolean isAtTargetPos(){
        return Math.abs(getPosError()) <= 1.2;
    }

    public void setDrive(double drive){
        elevatorLeftMotor.set(drive);
        elevatorRightMotor.set(drive);
    }

    public double getActualPos(){
        return leftPos.getValueAsDouble()-leftHomePos;
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
