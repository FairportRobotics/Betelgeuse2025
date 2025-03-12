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

  // Returns the value of the limit switch, True if hit
  public Boolean isCoralInHand(){
    return handLimitSwitch.get();
  }

  public void setSpeed(double handSpeed){
    handMotor.set(handSpeed);
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    Logger.recordOutput("Coral in hand", isCoralInHand());

    Logger.recordOutput("Hand speed", handMotor.get());
  }
  
  @Override
  // This method will be called once per scheduler run during simulation
  public void simulationPeriodic() {
  }
}
