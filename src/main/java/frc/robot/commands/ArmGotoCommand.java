// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.Constants.ArmPositions;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ArmGotoCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private ArmSubsystem _armSubsystem;
  private ArmPositions pos;
  private StatusSignal<Angle> currentPos;

  private StatusSignal<Double> posError;

  /**
   * Creates a new ArmGotoCommand.
   * This command sets the arm positon to the passed in position.
   * 
   * @param subsystem The ArmSubsystem. 
   * @param newPos The requested position of the arm. You can find what diffrent positions there are in Constants.java
   */
  public ArmGotoCommand(ArmSubsystem subsystem, ArmPositions newPos) {
    
    _armSubsystem = subsystem;
    pos = newPos;
    posError = _armSubsystem.getError();
    currentPos = _armSubsystem.getActualPos();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _armSubsystem.setTargetPos(pos);
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
    currentPos.refresh();
    boolean retVal = false;

        if (pos == ArmPositions.STOWED) {
            retVal = _armSubsystem.getSwitch();
        } 
        else if (currentPos.hasUpdated()) {

            SmartDashboard.putNumber("Arm Pos", currentPos.getValueAsDouble());

            retVal = (Math.abs(currentPos.getValueAsDouble() - (pos.getValue() )) <= 0.1);
        }

        return retVal;
  }
}
