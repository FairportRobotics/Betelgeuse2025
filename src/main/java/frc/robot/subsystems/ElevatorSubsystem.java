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
import frc.robot.Constants.ElevatorLevels;

public class ElevatorSubsystem extends TestableSubsystem {
  private ElevatorLevels goToLevel = ElevatorLevels.HOME;

  private double rightHomePos = Double.MAX_VALUE;
  private double leftHomePos = Double.MAX_VALUE;

  private TalonFX elevatorLeftMotor = new TalonFX(Constants.ElevatorMotors.LEFT_ID);
  private TalonFX elevatorRightMotor = new TalonFX(Constants.ElevatorMotors.RIGHT_ID);
  private DigitalInput bottomlimitSwitch;

  private StatusSignal<Angle> leftPos;
  private StatusSignal<Angle> rightPos;

  private StatusSignal<Double> leftError;
  private StatusSignal<Double> rightError;

  public ElevatorSubsystem(ArmSubsystem armSubsystem) {
    super("ElevatorSubsystem");

    // toplimitSwitch = new DigitalInput(8);
    bottomlimitSwitch = new DigitalInput(Constants.DIOValues.ELEVATORLIMIT);

    TalonFXConfiguration elevatorMotor1Config = new TalonFXConfiguration();
    elevatorMotor1Config.Slot0.kP = 0.7;
    elevatorMotor1Config.Slot0.kI = 0.5;
    elevatorMotor1Config.Slot0.kD = 0.1;
    elevatorMotor1Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    elevatorLeftMotor.getConfigurator().apply(elevatorMotor1Config);
    leftPos = elevatorLeftMotor.getPosition();
    leftError = elevatorLeftMotor.getClosedLoopError();
    leftPos.setUpdateFrequency(50);
    elevatorLeftMotor.optimizeBusUtilization();
    elevatorLeftMotor.setNeutralMode(NeutralModeValue.Brake);

    TalonFXConfiguration elevatorMotor2Config = new TalonFXConfiguration();
    elevatorMotor2Config.Slot0.kP = 0.7;
    elevatorMotor2Config.Slot0.kI = 0.5;
    elevatorMotor2Config.Slot0.kD = 0.1;
    elevatorMotor2Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    elevatorRightMotor.getConfigurator().apply(elevatorMotor2Config);
    rightPos = elevatorRightMotor.getPosition();
    rightError = elevatorRightMotor.getClosedLoopError();
    rightPos.setUpdateFrequency(50);
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

      this.elevatorLeftMotor.setNeutralMode(NeutralModeValue.Coast);
      this.elevatorRightMotor.setNeutralMode(NeutralModeValue.Coast);

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
    }

    Logger.recordOutput("Elevator At Bottom", !bottomlimitSwitch.get());

    Logger.recordOutput("Elevator Left Pos", leftPos.refresh().getValue());
    Logger.recordOutput("Elevator Right Pos", rightPos.refresh().getValue());
  }

  /**
   * Check for the level of elevator.
   * 
   * @return true if the elevator is currently at the correct level.
   */
  public boolean isAtLevel() {
    if (goToLevel == ElevatorLevels.HOME)
      return bottomlimitSwitch.get();
    return Math.abs(leftError.refresh().getValue()) < 0.1 || Math.abs(rightError.refresh().getValue()) < 0.1;
  }

  /**
   * Move the elevator to the desired level.
   * 
   * @param level The level to move the elevator to.
   */
  public void moveElevator(ElevatorLevels level) {
    if (leftHomePos == Double.MAX_VALUE || rightHomePos == Double.MAX_VALUE)
      return;
    Objects.requireNonNull(level, "level cannot be null");
    goToLevel = level;
    elevatorLeftMotor.setNeutralMode(NeutralModeValue.Coast);
    elevatorRightMotor.setNeutralMode(NeutralModeValue.Coast);
    if (level == ElevatorLevels.HOME) {
      elevatorLeftMotor.set(-0.1);
      elevatorRightMotor.set(-0.1);
    } else {
      elevatorLeftMotor.setControl(new PositionVoltage(leftHomePos + level.getRotationUnits()));
      elevatorRightMotor.setControl(new PositionVoltage(rightHomePos + level.getRotationUnits()));
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
    elevatorLeftMotor.setNeutralMode(NeutralModeValue.Brake);
    elevatorRightMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /**
   * Get the level the elevator is currently at.
   * 
   * @return The level the elevator is currently at.
   */
  public ElevatorLevels getGoToLevel() {
    return goToLevel;
  }
}
