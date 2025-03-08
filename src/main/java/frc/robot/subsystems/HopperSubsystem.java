// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.fairportrobotics.frc.posty.TestableSubsystem;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DIOValues;

public class HopperSubsystem extends TestableSubsystem {
  private DigitalInput beamBreak;
  private Command m_autoIntakeCommand;

  Alert alert = new Alert("Hopper beam is broken.", AlertType.kInfo);

  public HopperSubsystem(Command p_autoIntakeCommand) {
    super("HopperSubsystem");
    beamBreak = new DigitalInput(DIOValues.HOPPER_BEAM_BREAK_SENSOR);
    m_autoIntakeCommand = p_autoIntakeCommand;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run    
 
    alert.set(beamBreak.get());

    if (!beamBreak.get() && !m_autoIntakeCommand.isScheduled()) {
      //m_autoIntakeCommand.schedule();
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
