// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.DigestInputStream;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
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

  
  public SparkMax armYMotor; 
  public DigitalInput limitSwitch; 
  public double absPos;
  public ArmPositions pos; 
  public SparkClosedLoopController m_controller; 
  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    armYMotor = new SparkMax( Constants.ArmConstants.MotorYID , MotorType.kBrushless);
    limitSwitch = new DigitalInput(Constants.ArmConstants.LimitID);
    pos = ArmPositions.NONE;
    m_controller = armYMotor.getClosedLoopController();
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
    if(pos == ArmPositions.NONE){
            
            this.armYMotor.set(0.1);
            if (!this.limitSwitch.get()) {
                this.armYMotor.set(0.0);
                pos = ArmPositions.UP;
                armYMotor.getEncoder().setPosition(0);
                absPos = armYMotor.getEncoder().getPosition();
            }
      }
      Logger.recordOutput("Arm at Home ", !limitSwitch.get());
      Logger.recordOutput("Arm Position: ", armYMotor.getEncoder().getPosition());

  }
  /**
   * Get the value of the curent set position for the arm.
   *
   * @return an ArmPositions object that is curently set in the Subsystem. So you can know what position the arm is curently set to. It's kinda usefull.
   */
  public ArmPositions getPos(){
    return pos;
  }

  /**
   * Set the value of the arm position.
   *
   *@param newPos New ArmPositions object to go to. This is important for keeping track of where the arm is. Maybe.
   */
  public void setPos(ArmPositions newPos){
    m_controller.setReference(newPos.getValue(), ControlType.kPosition);
    pos = newPos;
  }



  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

  }
  
}
