package frc.robot.commands;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorUpCommand extends Command {
   ///**
   // * Creates a new TestElevatorUpCommand.
   // * Attempts to move the elevator up one level if possible.
   // * 
   // * @param elevatorSubsystem The elevator subsystem used by this command.
   // */
   public ElevatorUpCommand(ElevatorSubsystem elevatorSubsystem) {
 //     super(elevatorSubsystem);
        ElevatorSubsystem.elevatorLeftMotor.set(.1);
        ElevatorSubsystem.elevatorRightMotor.set(.1);
   }
   
   @Override
   public void execute() {
    //   goToLevel = getLevel(elevatorSubsystem.getGoToLevel());
    //   super.execute();
   }

   @Override
   public boolean isFinished() {
       return true;
   }

   @Override
    public void end(boolean interrupted) {
        ElevatorSubsystem.elevatorLeftMotor.stopMotor();
        ElevatorSubsystem.elevatorRightMotor.stopMotor();
        ElevatorSubsystem.elevatorLeftMotor.setNeutralMode(NeutralModeValue.Brake);
        ElevatorSubsystem.elevatorRightMotor.setNeutralMode(NeutralModeValue.Brake);
    }

   /**
   * Gets the next level of the elevator.
   * 
   * @param currentLevel is the current level of the elevator.
   * @return the next level of the elevator, returns the current level if the
   *         level of the elevator cannot be increased.
   */
//    private static ElevatorLevels getLevel(ElevatorLevels currentLevel) {
//       if (validToMoveUp(currentLevel))
//           return ElevatorLevels.values()[currentLevel.ordinal() + 1];
//       return currentLevel;
//   }
   //
   ///**
   // * Checks if the elevator is not at the top level.
   // * 
   // * @param currentLevel The current level of the elevator.
   // * @return true if the elevator is not at the top level, false otherwise.
   // */
   //private static boolean validToMoveUp(ElevatorLevels currentLevel) {
   //    return !ElevatorLevels.values()[ElevatorLevels.values().length - 1].equals(currentLevel);
   //}
}
