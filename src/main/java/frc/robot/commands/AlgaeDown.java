// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlgaeDown extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField", "unused"})
  private final AlgaeSubsystem m_subsystem;
  private StatusSignal posError;
  private StatusSignal pos;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlgaeDown(AlgaeSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    pos = m_subsystem.getPos();
    posError = m_subsystem.get_Error();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_subsystem.deploy_ballsucker();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
posError.refresh();
      pos.refresh();

        //if (pos == ArmPositions.UP) {
        //    return !m_subsystem.limitSwitch.get();
        //} 
        /*else /* */ if (pos.hasUpdated()) {
            SmartDashboard.putNumber("Arm Pos", pos.getValueAsDouble());
            return (Math.abs(pos.getValueAsDouble() - 1) <= 0.1);
        }

        return false;  }
}
