// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.Constants.ArmConstants.ArmPositions;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** An example command that uses an example subsystem. */
public class SetArmPosCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_subsystem;
  private ArmPositions pos;
  private StatusSignal<Angle> currentPos;

  private StatusSignal<Double> posError;

  final PositionVoltage posRequest;
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
    posError = m_subsystem.getError();

    posRequest = new PositionVoltage(0).withSlot(0);


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setPos(pos,posRequest);
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
    posError.refresh();

        if (pos == ArmPositions.UP) {
            return m_subsystem.getSwitch();
        } 
        else if (currentPos.hasUpdated()) {

            SmartDashboard.putNumber("Arm Pos", currentPos.getValueAsDouble());

            return (Math.abs(currentPos.getValueAsDouble() - (pos.getValue() )) <= 0.1);
        }

        return false;
  }
}
