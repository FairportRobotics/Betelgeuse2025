package frc.robot.subsystems;

import org.fairportrobotics.frc.posty.TestableSubsystem;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.DIOValues; 

public class HandSubsystem extends TestableSubsystem {

  
  public static Object getSwitch;
  private SparkMax handMotor = new SparkMax(Constants.CanBusIds.HAND_MOTOR_ID, MotorType.kBrushless);
  private DigitalInput handLimitSwitch = new DigitalInput(DIOValues.HAND_LIMIT_SWITCH);
  // public SparkClosedLoopController m_controller = handMotor.getClosedLoopController();
  public HandSubsystem() {
    super("HandSubsystem");

    registerPOSTTest("Motor is connected", () -> handMotor.getBusVoltage() > 0);
  }

  public Boolean isCoralInHand()
  {
    return handLimitSwitch.get();
  }
  //True or false from limit switch, its reverse so clear = false, obstructed = true its weird bro who ever designed this needs a new education

  public void setSpeed(double iShowSpeed){
    handMotor.set(iShowSpeed);
    if (iShowSpeed == 0) {
      handMotor.stopMotor();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    Logger.recordOutput("Coral in hand", isCoralInHand());

  }
  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

  }

}
