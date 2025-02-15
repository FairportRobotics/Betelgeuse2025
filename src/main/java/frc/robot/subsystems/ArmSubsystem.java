// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.DigestInputStream;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants.ArmPositions;

public class ArmSubsystem extends SubsystemBase {

  public TalonFX armYMotor;
  public DigitalInput limitSwitch;
  public StatusSignal absPos;
  public ArmPositions pos;
  final VelocityVoltage m_velocity = new VelocityVoltage(0);

  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    armYMotor = new TalonFX(4);
    limitSwitch = new DigitalInput(Constants.ArmConstants.LimitID);
    pos = ArmPositions.DOWN;

    TalonFXConfiguration armYConfig = new TalonFXConfiguration();
        armYConfig.Slot0.kP = 0.8;
        armYConfig.Slot0.kI = 0.5;
        armYConfig.Slot0.kD = 0.3;
        armYMotor.getConfigurator().apply(armYConfig);
        absPos = armYMotor.getPosition();
        absPos.setUpdateFrequency(50);
        armYMotor.optimizeBusUtilization();
    armYMotor.getConfigurator().apply(armYConfig, 0.050);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /*if (pos == ArmPositions.NONE) {

      this.armYMotor.set(0.1);
      if (!this.limitSwitch.get()) {
        this.armYMotor.set(0.0);
        pos = ArmPositions.UP;
        armYMotor.setPosition(0);
        absPos = armYMotor.getPosition();
      }
    }
    Logger.recordOutput("Arm at Home ", !limitSwitch.get());
    */
  }

  /**
   * Get the value of the curent set position for the arm.
   *
   * @return an ArmPositions object that is curently set in the Subsystem. So you
   *         can know what position the arm is curently set to. It's kinda
   *         usefull.
   */
  public ArmPositions getPos() {
    return pos;
  }

  /**
   * Set the value of the arm position.
   *
   * @param newPos New ArmPositions object to go to. This is important for keeping
   *               track of where the arm is. Maybe.
   */
  public void setPos(ArmPositions newPos, PositionVoltage PosRequest) {
    armYMotor.setNeutralMode(NeutralModeValue.Coast);

    absPos = armYMotor.getPosition();
    PosRequest = new PositionVoltage(0).withSlot(0);
    armYMotor.setControl(PosRequest.withPosition(pos.getValue()));
  }

  public void stopMotor(){
    armYMotor.stopMotor();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

  }

}
