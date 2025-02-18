// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ClimberPositions;
import frc.robot.Constants.ArmConstants.ArmPositions;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** An example command that uses an example subsystem. */
public class ClimberDown extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final ClimbingSubsystem m_subsystem;
  private ClimberPositions pos;
  private StatusSignal<Angle> currentPos;

  private StatusSignal<Double> posError;

  final PositionVoltage posRequest;

  /**
   * Creates a new ClimberDown command.
   * ClimberDown brings the Climber down.
   * It's quite self explanitorty.
   * 
   * @param subsystem The ClimbingSubsystem. If you don't give it this, it
   *                  explodes.
   */
  public ClimberDown(ClimbingSubsystem subsystem) {
    m_subsystem = subsystem;
    pos = ClimberPositions.DOWN;

    posRequest = new PositionVoltage(0).withSlot(0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPos = m_subsystem.getPos();

    posError = m_subsystem.getError();

    m_subsystem.setPos(pos, posRequest);
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
