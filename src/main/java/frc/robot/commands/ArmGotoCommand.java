// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ArmPositions;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmGotoCommand extends Command {
  private ArmSubsystem armSubsystem;
  private ArmPositions pos;

  /**
   * Creates a new ArmGotoCommand.
   * This command sets the arm positon to the passed in position.
   * 
   * @param subsystem The ArmSubsystem.
   * @param newPos    The requested position of the arm. You can find what
   *                  diffrent positions there are in Constants.java
   */
  public ArmGotoCommand(ArmSubsystem armSubsystem, ArmPositions newPos) {
    this.armSubsystem = armSubsystem;
    pos = newPos;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.setTargetPos(pos);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armSubsystem.isAtTargetPos();
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted)
      armSubsystem.stopMotor();
  }
}
