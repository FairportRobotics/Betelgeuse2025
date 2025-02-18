
package frc.robot.subsystems;


import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.jni.CANSparkJNI;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants.ArmPositions;

import com.revrobotics.spark.SparkMax;

public class AlgaeSubsystem extends SubsystemBase 
{//START
  private TalonFX kraken_Motor;
  private DigitalInput limitSwitch;
  private SparkMax wheelspin;
  private StatusSignal pos;



  public AlgaeSubsystem() 
  {//ALGAESUBSYSTEM
    kraken_Motor = new TalonFX(0);
    wheelspin = new SparkMax(0, null);//deal with paramaters later
    limitSwitch= new DigitalInput(0);
        //PID LOOP
        TalonFXConfiguration krakenConfig = new TalonFXConfiguration();
        krakenConfig.Slot0.kP = 0.8;
        krakenConfig.Slot0.kI = 0.5;
        krakenConfig.Slot0.kD = 0.3;
        kraken_Motor.getConfigurator().apply(krakenConfig);
        pos = kraken_Motor.getPosition();
        pos.setUpdateFrequency(50);
        kraken_Motor.optimizeBusUtilization();
        kraken_Motor.getConfigurator().apply(krakenConfig, 0.050);
        //PID LOOP
    
  }//ALGAESUBSYSTEM

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void deploy_ballsucker()
  {//BALLSUCKER
    wheelspin.set(.1);
    //this wheel is the wheel and it wheel the wheel?
    kraken_Motor.setNeutralMode(NeutralModeValue.Coast);
    PositionVoltage PosRequest = new PositionVoltage(0).withSlot(0);
    kraken_Motor.setControl(PosRequest.withPosition(1)); //probably less than that

  
  }//BALLSUCKERS

  public StatusSignal getPos()
  {
    return pos;
  }
  public void close_everything()
  {
    wheelspin.stopMotor();//i died its so difficult doing this
    kraken_Motor.set(-.1);




  }

  public void stop_kraken()
  {
    kraken_Motor.stopMotor();
  }

  public StatusSignal get_Error()
  {//Get_Error
    return kraken_Motor.getClosedLoopError();
  }//Get_Error

  public boolean get_switch()
  {//Get_Switch
    return !limitSwitch.get();
  }//Get_Switc

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition()
 {//NAME
    // Query some boolean state, such as a digital sensor.
    return false;
  }//NAME

  @Override
  public void periodic() 
  {//NAME
      if (pos == null) 
      {
        kraken_Motor.set(-.1);
        if (limitSwitch.get()) 
        {
          kraken_Motor.set(0.0);
          kraken_Motor.setPosition(0);
          pos = kraken_Motor.getPosition();
        }
    }
    Logger.recordOutput("Arm at Home ", !limitSwitch.get());  
  }//NAME

  @Override
  public void simulationPeriodic()
  {//NAME
    // This method will be called once per scheduler run during simulation
  }//NAME
  
}//END
