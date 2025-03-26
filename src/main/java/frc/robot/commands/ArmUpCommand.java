// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ArmPositions;
import frc.robot.subsystems.ArmSubsystem;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An ArmUpCommand that uses an example subsystem. */
public class ArmUpCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final ArmSubsystem m_subsystem;
  private ArmPositions pos;
  private StatusSignal<Angle> currentPos;
  private StatusSignal<Double> posError;

  /**
   * Creates a new ArmUpCommand.
   * ArmDownCommand brings the arm up 1 position if the current position is not
   * less than or equal to the UP positon
   * 
   * @param subsystem The ArmSubsystem.
   */
  public ArmUpCommand(ArmSubsystem subsystem) {
    m_subsystem = subsystem;
    System.out.println(m_subsystem.getArmPos());
    currentPos = m_subsystem.getActualPos();
 //    posError = m_subsystem.getPosError();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pos = m_subsystem.getArmPos();
    System.out.println("pos was " + pos);

    if (pos.ordinal() > ArmPositions.STOWED.ordinal()) {
      pos = ArmPositions.values()[pos.ordinal() - 1];
    }
    m_subsystem.setTargetPos(pos);
    System.out.println("pos is now" + pos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    posError.refresh();
    currentPos.refresh();

    // if (pos == ArmPositions.UP) {
    // return !m_subsystem.limitSwitch.get();
    // }
    /* else /* */ if (currentPos.hasUpdated()) {
      System.out.println("currentPos updated :D");
      SmartDashboard.putNumber("Arm Pos", currentPos.getValueAsDouble());
      System.out.println(currentPos.getValueAsDouble());
      System.out.println(Math.abs(currentPos.getValueAsDouble() - (pos.getValue())));
      return (Math.abs(currentPos.getValueAsDouble() - (pos.getValue())) <= 0.1);
    }

    return false;
  }
}
