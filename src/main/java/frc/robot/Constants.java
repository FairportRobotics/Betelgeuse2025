// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class AdvantageKitConstants{
    public enum RobotType{
      SIM,
      REAL,
      REPLAY
    }

    public static final RobotType CURRENT_MODE = RobotType.REAL;
  }

  public enum ElevatorLevels
  {
    NONE,
    HOME,
    CORAL,
    ONE,
    TWO,
    THREE,
    FOUR,
    MAX;
  }

  //HOME is the MIN level of the elevator.
  public static class ElevatorEncoderValues
  {
    public static final double HOME = 0;
    public static final double CORAL = 0;
    public static final double ONE = 0;
    public static final double TWO = 0;
    public static final double THREE = 0;
    public static final double FOUR = 0;
    public static final double MAX = 0;
  }

  public static class ElevatorMotors {
    public static final int LEFT = 0;
    public static final int RIGHT = 0;
    public static final int MIDDLE = 0;
  }

  public static class ElevatorLimitSwitches {
    public static final int MAIN = 0;
    public static final int MIDDLE = 0;
  }

  public static class ElevatorConstants{
    public static final int ELEVATOR_LEFT_MOTOR_ID = -99999; //ALL IDs ARE PLACEHOLDERS
    public static final int ELEVATOR_RIGHT_MOTOR_ID = -999999;

    public static final int ELEVATOR_BOTTOM_SWITCH_ID = -999999999;
  }
}
