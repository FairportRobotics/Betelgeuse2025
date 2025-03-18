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
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.CanBusIds;
import frc.robot.Constants.ClimberPositions;
import frc.robot.Constants.DIOValues;

public class ClimbingSubsystem extends TestableSubsystem {

  private TalonFX climbingMotor;
  private DigitalInput limitSwitch;
  private StatusSignal<Angle> absPos;
  private StatusSignal<Double> error;
  private ClimberPositions pos;
  private final PositionVoltage m_position = new PositionVoltage(0).withSlot(0);

  /** Creates a new ClimbingSubsystem. */
  public ClimbingSubsystem() {
    super("ClimbingSubsystem");
    climbingMotor = new TalonFX(CanBusIds.CLIMBER_MOTOR_ID, "rio"); 
    climbingMotor.setNeutralMode(NeutralModeValue.Brake);
    limitSwitch = new DigitalInput(DIOValues.CLIMBER_LIMIT_SWITCH);
    pos = ClimberPositions.NONE;

    TalonFXConfiguration armYConfig = new TalonFXConfiguration();
        armYConfig.Slot0.kP = 0.3;
        armYConfig.Slot0.kI = 0.5;
        armYConfig.Slot0.kD = 0.3;
        climbingMotor.getConfigurator().apply(armYConfig);
        absPos = climbingMotor.getPosition();
        absPos.setUpdateFrequency(50);
        climbingMotor.optimizeBusUtilization();
    climbingMotor.getConfigurator().apply(armYConfig, 0.050);

    error = climbingMotor.getClosedLoopError();
    // registerPOSTTest("Motor is connected", () -> climbingMotor.isConnected());
  }


  @Override
  public void periodic() {
    if (pos == ClimberPositions.NONE) {
      climbingMotor.set(0.1);
      if (this.limitSwitch.get()) {
        this.climbingMotor.set(0.0);
        pos = ClimberPositions.HOME;
        climbingMotor.setPosition(0);
        absPos = climbingMotor.getPosition();
        absPos.setUpdateFrequency(10);
        error = climbingMotor.getClosedLoopError();
        error.setUpdateFrequency(10);
      }
    }
    Logger.recordOutput("Climber at home", isAtHome());
  }

  public boolean isAtHome(){
    return limitSwitch.get();
  }

  /**
   * Get the current position of the motor
   *
   * @return The position of the motor. It's not computer science. Wait...
   */
  public StatusSignal<Angle> getPos() {
    return absPos;
  }
 /**
   * Get the closed loop error for the motor.
   *
   * @return the closed loop error. You need this for the PID loop. I blame Jordan.
   */
  public StatusSignal<Double> getError(){
    return error;
  }

  /**
   * Set the position for the climber.
   *
   * @param newPos The new ClimberPositions to go to. Big Tyler is watching.
   */
  public void setPos(ClimberPositions newPos) {
    pos = newPos;
    absPos = climbingMotor.getPosition();
    climbingMotor.setControl(m_position.withPosition(pos.getValue()));
  }

  /**
   * Take a wild guess
   */
  public void stopMotor(){
    climbingMotor.stopMotor();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

  }

}
