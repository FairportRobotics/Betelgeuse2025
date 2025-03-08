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

public class ArmSubsystem extends TestableSubsystem {

  private TalonFX armYMotor;
  private DigitalInput topSwitch; //Today on TopSwitch...
  private StatusSignal<Angle> actualPos;
  public double armHomePos = Double.MAX_VALUE;
  private ArmPositions targetPos;
  private final PositionVoltage m_voltage = new PositionVoltage(0).withSlot(0);
  private ElevatorSubsystem mElevatorSubsystem;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    super("ArmSubsystem");
    armYMotor = new TalonFX(CanBusIds.ARM_MOTOR_ID, "rio"); // TODO: FIX ID
    armYMotor.setNeutralMode(NeutralModeValue.Brake);
    topSwitch = new DigitalInput(DIOValues.ARM_LIMIT_SWITCH);
    targetPos = ArmPositions.NONE;

    TalonFXConfiguration armYConfig = new TalonFXConfiguration();
    armYConfig.Slot0.kP = 1;
    armYConfig.Slot0.kI = 0.5;
    armYConfig.Slot0.kD = 0.3;
    armYConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    armYMotor.getConfigurator().apply(armYConfig);
    actualPos = armYMotor.getPosition();
    actualPos.setUpdateFrequency(50);
    armYMotor.optimizeBusUtilization();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (armHomePos == Double.MAX_VALUE) {

      this.armYMotor.set(0.1);
      if (getSwitch()) {
        this.armYMotor.set(0.0);
        
        StatusSignal<Angle> pos = armYMotor.getPosition();

        actualPos.waitForUpdate(1.0);

        armHomePos = actualPos.getValueAsDouble();

        this.armYMotor.setNeutralMode(NeutralModeValue.Brake);
      }
    }
    Logger.recordOutput("Arm at Home ", !topSwitch.get());
    
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

}
