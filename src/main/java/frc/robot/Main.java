// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  private static Boolean robotWorks;
  private Main() {}

  public static void main(String... args) throws RobotDontWorkException {
    robotWorks = true;
    if (robotWorks) {
      RobotBase.startRobot(Robot::new);
    }
    else {
      throw new RobotDontWorkException("Robot don't work");
    }
  }

  public static class RobotDontWorkException extends Exception
{
      // Parameterless Constructor
      public RobotDontWorkException() {}

      // Constructor that accepts a message
      public RobotDontWorkException(String message)
      {
         super(message);
      }
 }
}
