// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HandSubsystem;

public class HandCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  HandSubsystem _HandSubsystem;
  double speed;
  VelocityVoltage intakeRequest;

  /**
   * Creates a new HandCommand.
   * HandCommand causees the neo in the hand to spin off the given value either
   * intaking or outaking (positive for intake, negative for outake)
   * 
   * @param handSubsystem The HandSubsystem.
   * @param intakeSpeed
   */
  public HandCommand(HandSubsystem handSubsystem, double intakeSpeed) {
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
    if(_HandSubsystem.isCoralInHand() && speed < 0)
      return true;
    else{
      return false;
    }
  }

  // Stops the movement of the motor. 
  @Override
  public void end(boolean interrupted) {
    _HandSubsystem.setSpeed(0.0);
  }
}
