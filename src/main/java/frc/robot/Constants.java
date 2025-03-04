// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class HandConstants {
    public static final int HAND_MOTOR_ID = 16;
    public static final int HAND_LIMIT_SWITCH_ID = 1;
  }

  public enum ClimberPositions {
    IN(1),
    OUT(2),
    HOME(3),
    NONE(0);

    double wa;

    private ClimberPositions(double value) {
      this.wa = value;
    }
    /**
       * Get the value of the ArmPositions Object
       * 
       * @return A double that is to be used for seting the position of the arm.
       */
      public double getValue() {
        return wa;
      }
  }

  public static class ArmConstants {

    public static final int MotorYID = 15;
    public static final int LimitID = 0;

    /**
     * The ArmPositions Enum is used to store positons for the arm. There are 4
     * values.
     * - UP 
     * - MIDDLE
     * - DOWN
     * - NONE
     * Each has a double that can be accesed with ArmPositions.getValue. Each value
     * only has one double. No more, no less. If thou shall try to get two, thou
     * shall only get one, and one shall be the number given. Trying to get three is
     * outright.
     * 
     */
    public enum ArmPositions {
      UP(1),
      MIDDLE(2),
      DOWN(3),
      NONE(0);

      double wa;

      private ArmPositions(double value) {
        this.wa = value;
      }

      /**
       * Get the value of the ArmPositions Object
       * 
       * @return A double that is to be used for seting the position of the arm.
       */
      public double getValue() {
        return wa;
      }
    }

    public static class AdvantageKitConstants {
      public enum RobotType {
        SIM,
        REAL,
        REPLAY
      }

      public static final RobotType CURRENT_MODE = RobotType.REAL;
    }

  }

  public enum ElevatorLevels {
    HOME,
    CORAL,
    ONE,
    TWO,
    THREE,
    FOUR,
    // MAX;
  }

  public static class DIOValues {
    public static final int ALGAELIMIT = 0;
    public static final int ARMLIMIT = 1;
    public static final int CLIMBERLIMIT = 2;
    public static final int ELEVATORLIMIT = 3;
    public static final int HANDLIMIT = 4;
    public static final int HOPPERBEAM = 5;
  }

  // HOME is the MIN level of the elevator.
  public static class ElevatorEncoderValues {
    public static final double HOME = 0;
    public static final double CORAL = 3;
    public static final double ONE = 6;
    public static final double TWO = 9;
    public static final double THREE = 12;
    public static final double FOUR = 15;
    // public static final double MAX = 19;
  }

  public static class ElevatorMotors {
    public static final int LEFT_ID = 14;
    public static final int RIGHT_ID = 13;
  }

  public static class ElevatorLimitSwitches {
    public static final int BOTTOM_ID = 9;
  }
}
