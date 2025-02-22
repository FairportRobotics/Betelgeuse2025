package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DIOValues; 

public class HandSubsystem extends SubsystemBase {

  
  public static Object getSwitch;
  private SparkMax handMotor = new SparkMax(Constants.HandConstants.HAND_MOTOR_ID, MotorType.kBrushless);
  private DigitalInput handLimitSwitch = new DigitalInput(DIOValues.HANDLIMIT);
  // public SparkClosedLoopController m_controller = handMotor.getClosedLoopController();
  private boolean hazPiece;
  /** Creates a new ExampleSubsystem. */
  public HandSubsystem() {
    
  }

  public Boolean getSwitch()
  {
    return !handLimitSwitch.get();
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
    
  }
  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

  }
  
}
