// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.DigestInputStream;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants.ArmPositions;


public class ArmSubsystem extends SubsystemBase {

  
  public SparkMax armYMotor = new SparkMax(0, MotorType.kBrushless);
  public DigitalInput limitSwitch = new DigitalInput(0);
  public double absPos;
  public ArmPositions pos = ArmPositions.NONE;
  public SparkClosedLoopController m_controller = armYMotor.getClosedLoopController();
  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    
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

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(pos == ArmPositions.NONE){
            
            this.armYMotor.set(0.1);
            if (!this.limitSwitch.get()) {
                this.armYMotor.set(0.0);
                pos = ArmPositions.UP;
                armYMotor.getEncoder().setPosition(0);
                absPos = armYMotor.getEncoder().getPosition();
            }
      }
      Logger.recordOutput("Elevator At Top: ", !limitSwitch.get());
      Logger.recordOutput("Arm Position: ", armYMotor.getEncoder().getPosition());

  }
  public ArmPositions getPos(){
    return pos;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

  }
  
}
