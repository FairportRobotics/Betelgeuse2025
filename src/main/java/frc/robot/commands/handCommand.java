// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
 

public class handCommand extends Command{
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    HandSubsystem _HandSubsystem;
    double speed;
    VelocityVoltage intakeRequest;
    public DigitalInput handLimitSwitch = new DigitalInput(1);
   /**
   * Creates a new HandCommand.
   *  HandCommand causees the neo in the hand to spin off the given value either intakeing or outakeing
   * @param subsystem The HandSubsystem.
   */
    public handCommand(HandSubsystem handSubsystem, double intakeSpeed){
        _HandSubsystem = handSubsystem;
        speed = intakeSpeed;
        addRequirements(_HandSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      _HandSubsystem.setSpeed(speed);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
    return handLimitSwitch.get();
    }

    //Ends the movement of the motor
    @Override
    public void end(boolean interrupted) {
        _HandSubsystem.setSpeed(0.0);
    }
}