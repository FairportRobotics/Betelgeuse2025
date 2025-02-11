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
public class ArmDownCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_subsystem;
  private ArmPositions pos;
  /**
   * Creates a new ArmDownCommand.
   *  ArmDownCommand brings the arm down 1 position if the current position is not less than or equal to the DOWN positon
   * 
   * @param subsystem The ArmSubsystem. You know... the thing... that does... the thing...
   */
  public ArmDownCommand(ArmSubsystem subsystem) {
    m_subsystem = subsystem;
    pos = m_subsystem.getPos();
    if (pos.ordinal() < ArmPositions.DOWN.ordinal()) {
      pos = ArmPositions.values()[pos.ordinal() - 1];
    }


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_subsystem.m_controller.setReference(pos.getValue(), ControlType.kPosition);
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
