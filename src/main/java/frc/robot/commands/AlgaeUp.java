// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.AlgaeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An AlgaeUp command that uses the Algae subsystem. */
public class AlgaeUp extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final AlgaeSubsystem a_subsystem;

  /**
   * Creates a new AlgaeUp.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlgaeUp(AlgaeSubsystem subsystem) {
    a_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(a_subsystem.getSwitch() == false)//run if not up
    {
      a_subsystem.closeBallIntake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    a_subsystem.stopFenceMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return a_subsystem.getSwitch();
  }
}
