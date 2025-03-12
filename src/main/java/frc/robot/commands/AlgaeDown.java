// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.AlgaeSubsystem;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An AlgaeDown command that uses the Algae subsystem. */
public class AlgaeDown extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final AlgaeSubsystem a_subsystem;
  private StatusSignal<Double> posError;
  private StatusSignal<Angle> pos;
  /**
   * Creates a new AlgaeDown.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlgaeDown(AlgaeSubsystem subsystem) 
  {
    a_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    pos = a_subsystem.getPos();
    posError = a_subsystem.getError();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    a_subsystem.openBallIntake();
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
