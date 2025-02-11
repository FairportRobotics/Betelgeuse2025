// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.Constants.ArmConstants.ArmPositions;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** An example command that uses an example subsystem. */
public class SetArmPosCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_subsystem;
  private ArmPositions pos;
  /**
   * Creates a new SetArmPosCommand.
   * This command sets the arm positon to the passed in position.
   * 
   * @param subsystem The ArmSubsystem. This is needed. Because. Just because.
   * @param newPos The requested position of the arm. You can find what diffrent positions there are in Constants.java
   */
  public SetArmPosCommand(ArmSubsystem subsystem, ArmPositions newPos) {
    m_subsystem = subsystem;
    pos = newPos;



    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setPos(pos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_subsystem.armYMotor.getEncoder().getPosition() == pos.getValue()) {
      return true;
    }
    return false;
  }
}
