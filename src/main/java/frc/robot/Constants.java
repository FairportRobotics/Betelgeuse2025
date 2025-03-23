// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

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
    public static final int ALGAE_LIMIT_SWITCH        = 15; // NOT DEFINED
    public static final int ARM_LIMIT_SWITCH          = 0;
    public static final int HAND_LIMIT_SWITCH         = 1;
    public static final int ELEVATOR_LIMIT_SWITCH     = 3;
    public static final int CLIMBER_LIMIT_SWITCH      = 4;
    public static final int HOPPER_BEAM_BREAK_SENSOR  = 2;
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
    STOWED(0),
    HOME(3),
    MIDDLE(23),
    DOWN(52),
    NONE(0),
    SCORINGTOP(14),
    SCORINGBOTTOM(20);

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
    HUMAN_PLAYER_STATION(-9),
    ARM_LIMIT(-4),  // lowest elevator position with arm down
    ONE(-1),
    TWO(-4.8),
    THREE(-10.5),
    FOUR(-19);

    double mElevatorPosition;

    private ElevatorPositions(double rotationUnits) {
      mElevatorPosition = rotationUnits;
    }

    public double getRotationUnits() {
      return mElevatorPosition;
    }
  }


    public enum DriveWaypoints {
        REEF_A("A"),
        REEF_B("B"),
        REEF_C("C"),
        REEF_D("D"),
        REEF_E("E"),
        REEF_F("F"),
        REEF_G("G"),
        REEF_H("H"),
        REEF_I("I"),
        REEF_J("J"),
        REEF_K("K"),
        REEF_L("L"),
        REEF_A_BACK("A Back"),
        REEF_B_BACK("B Back"),
        REEF_C_BACK("C Back"),
        REEF_D_BACK("D Back"),
        REEF_E_BACK("E Back"),
        REEF_F_BACK("F Back"),
        REEF_G_BACK("G Back"),
        REEF_H_BACK("H Back"),
        REEF_I_BACK("I Back"),
        REEF_J_BACK("J Back"),
        REEF_K_BACK("K Back"),
        REEF_L_BACK("L Back"),

        HPS_LEFT("LeftPS"),
        HPS_RIGHT("RightPS");

        public String pathName;

        private DriveWaypoints( String pathName ){
            this.pathName = pathName;
        }
    }


}
