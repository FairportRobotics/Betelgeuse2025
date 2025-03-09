// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.CanBusIds;
import frc.robot.Constants.DIOValues;
import frc.robot.Constants.ElevatorPositions;

public class ArmSubsystem extends TestableSubsystem {

  private final double DEFAULT_HOME_POS = 0.00001;
  public double armHomePos = DEFAULT_HOME_POS;

  private TalonFX armYMotor;
  private DigitalInput topSwitch; //Today on TopSwitch...
  private StatusSignal<Angle> actualPos;
  private StatusSignal<Double> requestedPos;
  private ArmPositions targetPos;
  private final PositionVoltage m_voltage = new PositionVoltage(0).withSlot(0);
  private ElevatorSubsystem mElevatorSubsystem;
  private double lowestValidArmPosition = ArmPositions.MIDDLE.getValue();

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    super("ArmSubsystem");
    armYMotor = new TalonFX(CanBusIds.ARM_MOTOR_ID, "rio");
    armYMotor.setNeutralMode(NeutralModeValue.Brake);
    topSwitch = new DigitalInput(DIOValues.ARM_LIMIT_SWITCH);
    targetPos = ArmPositions.NONE;

    TalonFXConfiguration armYConfig = new TalonFXConfiguration();
    armYConfig.Slot0.kP = .5;
    armYConfig.Slot0.kI = 0.2;
    armYConfig.Slot0.kD = 0.1;
    armYConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    armYConfig.CurrentLimits.StatorCurrentLimit = 30;
    armYConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    armYMotor.getConfigurator().apply(armYConfig);
    actualPos = armYMotor.getPosition();
    actualPos.setUpdateFrequency(50);
    requestedPos = armYMotor.getClosedLoopReference();
    requestedPos.setUpdateFrequency(50);
    armYMotor.optimizeBusUtilization();

    registerPOSTTest("Arm Motor Connected", () -> {
            return armYMotor.isConnected();
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (armHomePos == DEFAULT_HOME_POS) {

      if (getSwitch()) {
        this.armYMotor.set(0.0);

        StatusSignal<Angle> pos = armYMotor.getPosition();

        actualPos.waitForUpdate(1.0);

        armHomePos = actualPos.getValueAsDouble();

        this.armYMotor.setNeutralMode(NeutralModeValue.Brake);
        return;
      }
      this.armYMotor.set(-.1);
    }
    Logger.recordOutput("Arm at Home ", getSwitch());

    Logger.recordOutput("Arm Pos", actualPos.refresh().getValueAsDouble()-armHomePos);

    Logger.recordOutput("Arm Requested Pos", requestedPos.refresh().getValueAsDouble()-armHomePos);

  }

  /**
   * Get the value of the current set position for the arm.
   *
   * @return an ArmPositions object that is currently set in the Subsystem. So you
   *         can know what position the arm is currently set to. It's kinda
   *         useful.
   */
  public ArmPositions getArmPos() {
    return targetPos;
  }

  /**
   * Get the value of the current position of the motor.
   *
   * @return The current position of the motor.
   */
  public StatusSignal<Angle> getActualPos() {
    return actualPos;
  }

  /**
   * Get the closed loop error of the motor.
   *
   * @return motor.getClosedLoopError. It's as shrimple as that
   */
  public StatusSignal<Double> getError() {
    return armYMotor.getClosedLoopError();
  }

  /**
   * The value of the limitswitch
   *
   * @return True when switch is triggered, False when not. 
   */
  public boolean getSwitch() {
    return topSwitch.get();
  }

  /**
   * Set the value of the arm position.
   *
   * @param newPos New ArmPositions object to go to. This is important for keeping
   *               track of where the arm is. Maybe.
   */
  public void setTargetPos(ArmPositions newPos) {
    targetPos = newPos;
    actualPos = armYMotor.getPosition();
    armYMotor.setControl(m_voltage.withPosition(targetPos.getValue() + armHomePos));
  }

  /**
   * What do you think this does?
   */
  public void stopMotor() {
    armYMotor.stopMotor();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

  }

  public void setElevatorSubsystem(ElevatorSubsystem theElevatorSubsytem){
    mElevatorSubsystem = theElevatorSubsytem;
  }

  public boolean canGoToPosition(ArmPositions requestedPos){
    if (requestedPos.getValue() > lowestValidArmPosition)
        return true;
    else
        return false;
  }

}
