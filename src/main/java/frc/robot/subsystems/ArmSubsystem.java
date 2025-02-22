// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants.ArmPositions;

public class ArmSubsystem extends SubsystemBase {

  private TalonFX armYMotor;
  private DigitalInput limitSwitch;
  private StatusSignal<Angle> absPos;
  private ArmPositions pos;
  private final PositionVoltage m_voltage = new PositionVoltage(0).withSlot(0);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armYMotor = new TalonFX(4, "rio");
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


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (pos == ArmPositions.NONE) {

      this.armYMotor.set(0.1);
      if (!this.limitSwitch.get()) {
        this.armYMotor.set(0.0);
        pos = ArmPositions.UP;
        armYMotor.setPosition(0);
        absPos = armYMotor.getPosition();
        absPos.setUpdateFrequency(10);

      }
    }
    Logger.recordOutput("Arm at Home ", !limitSwitch.get());
    if (!limitSwitch.get() && armYMotor.getAcceleration().getValueAsDouble() <= 0.0) {
      stopMotor();
    }
  }

  /**
   * Get the value of the curent set position for the arm.
   *
   * @return an ArmPositions object that is curently set in the Subsystem. So you
   *         can know what position the arm is curently set to. It's kinda
   *         usefull.
   */
  public ArmPositions getArmPos() {
    return pos;
  }

    /**
   * Get the value of the curent position of the motor.
   *
   * @return The current position of the motor. I literaly just said it.
   */
  public StatusSignal<Angle> getPos(){
    return absPos;
  }

      /**
   * Get the closed loop error of the motor.
   *
   * @return motor.getClosedLoopError. It's as shrimple as that
   */
  public StatusSignal<Double> getError(){
    return armYMotor.getClosedLoopError();
  }

      /**
   * The value of the limitswitch
   *
   * @return True when switch is triggered, False when not. It originaly did the opposite and it's so stupid. You would think naturaly not triggered would be false, but NOOOOO. That was just too much to ask for from our limit switch. It just thinks it's so SMART by mixing us up. When nanson was working on the elevator code, he spend a good few minuets trying to figure out why the homing code would not work. When he figured out that the limit switches were returning false, he did a backflip so large, he made it to the moon. It took us 13.4 Billion dollars to get him back(shout out to our sponsors) and once we did, he told us all about how aliens were making a colony there and how it was made of chesse and how the limit swich returned false when not triggered. We couldn't belive our ears(mostly because of the moon stuff).
   */
  public boolean getSwitch(){
    return !limitSwitch.get();
  }


  /**
   * Set the value of the arm position.
   *
   * @param newPos New ArmPositions object to go to. This is important for keeping
   *               track of where the arm is. Maybe.
   */
  public void setPos(ArmPositions newPos) {
    armYMotor.setNeutralMode(NeutralModeValue.Coast);
    pos = newPos;
    absPos = armYMotor.getPosition();
    armYMotor.setControl(m_voltage.withPosition(pos.getValue()));
  }

  /**
   * What do you think this does?
   */
  public void stopMotor(){
    armYMotor.stopMotor();
  }



  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

  }

}
