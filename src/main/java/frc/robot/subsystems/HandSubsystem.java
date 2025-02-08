package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HandSubsystem extends SubsystemBase {

  
  public SparkMax handMotor = new SparkMax(10, MotorType.kBrushless);
  public DigitalInput handLimitSwitch = new DigitalInput(1);
  // public SparkClosedLoopController m_controller = handMotor.getClosedLoopController();
  private boolean hazPiece;
  /** Creates a new ExampleSubsystem. */
  public HandSubsystem() {
    
  }

  public void setSpeed(double iShowSpeed){
    this.handMotor.set(iShowSpeed);
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
