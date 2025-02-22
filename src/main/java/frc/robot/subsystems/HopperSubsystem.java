// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DIOValues;
import frc.robot.Constants.ElevatorLevels;
import frc.robot.commands.ElevatorGoToLevelCommand;
import frc.robot.subsystems.ElevatorSubsystem;

public class HopperSubsystem extends SubsystemBase {
  private DigitalInput beamBreak;
  private Command m_autoIntakeCommand;

  /** Creates a new ExampleSubsystem. */
  public HopperSubsystem(Command p_autoIntakeCommand) {
    beamBreak = new DigitalInput(DIOValues.HOPPERBEAM);
    m_autoIntakeCommand = p_autoIntakeCommand;

  }


  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!beamBreak.get() && !m_autoIntakeCommand.isScheduled()) {
      m_autoIntakeCommand.schedule();
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
