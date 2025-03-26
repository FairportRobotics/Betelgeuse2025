package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import org.w3c.dom.ElementTraversal;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.HopperSubsystem;

public class IntakeCoralCommand extends Command{

    private enum StateStep{
        NONE,
        ELEVATOR_HPS,
        ARM_READY,
        GRAB_CORAL,
        CORAL_IN_HAND,
        ELEVATOR_AT_SCORE_POS,
        READY_TO_SCORE
    };

    private ArmSubsystem armSubsystem;
    private HandSubsystem handSubsystem;
    private ElevatorSubsystem elevatorSubsystem;
    private HopperSubsystem hopperSubsystem;

    StateStep currentStep = StateStep.NONE;

    boolean isReady = false;

    public IntakeCoralCommand(ArmSubsystem armSub, HandSubsystem handSub, ElevatorSubsystem elevatorSub, HopperSubsystem hopperSub){
        this.armSubsystem = armSub;
        this.handSubsystem = handSub;
        this.elevatorSubsystem = elevatorSub;
        this.hopperSubsystem = hopperSub;

        this.addRequirements(armSubsystem, handSubsystem, elevatorSubsystem, hopperSubsystem);
    }

    @Override
    public void initialize() {
        currentStep = StateStep.NONE;
    }

    @Override
    public void execute() {

        Logger.recordOutput("Current Intake State", currentStep.name());
        SmartDashboard.putString("Current Intake State", currentStep.name());
        
        switch (currentStep) {
            case NONE:
                elevatorSubsystem.goToPosition(ElevatorPositions.HUMAN_PLAYER_STATION);

                if(elevatorSubsystem.isAtTargetPos()){
                    currentStep = StateStep.ELEVATOR_HPS;
                }

                break;

            case ELEVATOR_HPS:

                armSubsystem.setTargetPos(ArmPositions.DOWN);

                if(armSubsystem.isAtTargetPos()){
                    currentStep = StateStep.ARM_READY;
                }

                break;
            case ARM_READY:

                if(hopperSubsystem.isCoralInHopper()){
                    armSubsystem.setBrakeMode(false);
                    currentStep = StateStep.GRAB_CORAL;
                }

                break;
            case GRAB_CORAL:

                handSubsystem.setSpeed(-0.3);
                elevatorSubsystem.setDrive(0.02);

                if(elevatorSubsystem.getActualPos() >= -6.5){ // Failed, abort
                  elevatorSubsystem.setDrive(0);
                  elevatorSubsystem.goToPosition(ElevatorPositions.FOUR);
                  handSubsystem.setSpeed(0);
                  currentStep = StateStep.NONE;
                  break;
                }

                if(handSubsystem.isCoralInHand()){
                    elevatorSubsystem.setDrive(0);
                    handSubsystem.setSpeed(0);
                    currentStep = StateStep.CORAL_IN_HAND;
                }

                break;
            case CORAL_IN_HAND:

                elevatorSubsystem.goToPosition(ElevatorPositions.THREE);

                if(elevatorSubsystem.getActualPos() <= -7){
                    armSubsystem.setBrakeMode(true);
                    currentStep = StateStep.ELEVATOR_AT_SCORE_POS;
                    break;
                }

                break;
            case ELEVATOR_AT_SCORE_POS:

                armSubsystem.setTargetPos(ArmPositions.SCORINGTOP);

                if(armSubsystem.isAtTargetPos()){
                    currentStep = StateStep.READY_TO_SCORE;
                }

            case READY_TO_SCORE:
                break;
            default:
                break;
        }

    }

    @Override
    public boolean isFinished() {
        return currentStep == StateStep.READY_TO_SCORE;
    }

    @Override
    public void end(boolean interrupted) {
    }

}
