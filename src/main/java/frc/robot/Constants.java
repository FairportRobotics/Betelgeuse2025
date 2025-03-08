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

  public static class AdvantageKitConstants {
    public enum RobotType {
      SIM,
      REAL,
      REPLAY
    }

    public static final RobotType CURRENT_MODE = RobotType.REAL;
  }

  public static class CanBusIds {
    public static final int ELEVATOR_RIGHT_MOTOR_ID = 13;
    public static final int ELEVATOR_LEFT_MOTOR_ID  = 14;
    public static final int ARM_MOTOR_ID            = 15;
    public static final int HAND_MOTOR_ID           = 16;
    public static final int CLIMBER_MOTOR_ID        = 19;
  }

  public static class DIOValues {
    public static final int ALGAE_LIMIT_SWITCH        = 0; // NOT DEFINED
    public static final int ARM_LIMIT_SWITCH          = 0; // NOT DEFINED
    public static final int HAND_LIMIT_SWITCH         = 1;
    public static final int ELEVATOR_LIMIT_SWITCH     = 2;
    public static final int CLIMBER_LIMIT_SWITCH      = 4;
    public static final int HOPPER_BEAM_BREAK_SENSOR  = 5;
  }

  public static class ControllerIds {
    public static final int DRIVER_CONTROLLER_PORT    = 0;
    public static final int OPERATOR_CONTROLLER_PORT  = 1;
  }

  public enum ClimberPositions {
    IN(1),
    OUT(20),
    HOME(3),
    NONE(0);

    double mClimberPosition;

    private ClimberPositions(double value) {
      mClimberPosition = value;
    }

    /**
     * Get the value of the ClimberPositions Object
     * 
     * @return A double that is to be used for seting the position of the climber.
     */
    public double getValue() {
      return mClimberPosition;
    }
  }

  /**
   * The ArmPositions Enum is used to store positons for the arm.
   */
  public enum ArmPositions {
    UP(0),
    MIDDLE(.25),
    DOWN(.5),
    NONE(0);

    double mArmPosition;

    private ArmPositions(double value) {
      mArmPosition = value;
    }

    /**
     * Get the value of the ArmPositions Object
     * 
     * @return A double that is to be used for seting the position of the arm.
     */
    public double getValue() {
      return mArmPosition;
    }
  }

  /** Defines the rotationUnits required to get to each level from
   * the HOME position
   */
  public enum ElevatorPositions {
    HOME(0),
    HUMAN_PLAYER_STATION(4),
    ARM_LIMIT(4),  // lowest elevator position with arm down
    ONE(5),
    TWO(7),
    THREE(10),
    FOUR(15);

    double mElevatorPosition;

    private ElevatorPositions(double rotationUnits) {
      mElevatorPosition = rotationUnits;
    }

    public double getRotationUnits() {
      return mElevatorPosition;
    }

}
}
