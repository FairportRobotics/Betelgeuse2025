
package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.DIOValues;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;

public class AlgaeSubsystem extends SubsystemBase 
{//START

  //VARIABLES
  private TalonFX krakenMotor;
  private DigitalInput limitSwitch;
  private SparkMax wheelSpin;
  private StatusSignal<Angle> pos;
  //VARIABLES

  public AlgaeSubsystem() 
  {//ALGAESUBSYSTEM

    krakenMotor = new TalonFX(0);
    wheelSpin = new SparkMax(0, null);//deal with paramaters later
    limitSwitch= new DigitalInput(DIOValues.ALGAELIMIT);
       //PID LOOP
      TalonFXConfiguration krakenConfig = new TalonFXConfiguration();
      krakenConfig.Slot0.kP = 0.8;
      krakenConfig.Slot0.kI = 0.5;
      krakenConfig.Slot0.kD = 0.3;
      krakenMotor.getConfigurator().apply(krakenConfig);
      pos = krakenMotor.getPosition();
      pos.setUpdateFrequency(50);
      krakenMotor.optimizeBusUtilization();
      krakenMotor.getConfigurator().apply(krakenConfig, 0.050);
      //PID LOOP
    
  }//ALGAESUBSYSTEM

   /**
   * Spins the wheels(53)<P>
   * brings down the entire fence(56)<P>
   * @Note line 58 may change position
   */
  public void ballIntake()
  {//BallIntake
    wheelSpin.set(.1); 
    krakenMotor.setNeutralMode(NeutralModeValue.Coast);
    PositionVoltage PosRequest = new PositionVoltage(0).withSlot(0);
    krakenMotor.setControl(PosRequest.withPosition(1)); 
  }//BallIntake

   /**
   * @return Position
   */
  public StatusSignal<Angle> getPos()
  {//GetPos
    return pos;
  }//GetPos
  
  /**
   * Stops the wheels from spinning (74)<P>
   * Sets the kraken motor speed to -.1 so it closes(75)
   */
  public void closeIntake()
  {//CloseAlgae
    wheelSpin.stopMotor();
    krakenMotor.set(-.1);
  }//CloseAlgae

  /**
   * Stops the kraken motor
   */
  public void stopKraken()
  {//StopKraken
    krakenMotor.stopMotor();
  }//StopKraken

  /**
   * @return Error
   */
  public StatusSignal<Double> getError()
  {//GetError
    return krakenMotor.getClosedLoopError();
  }//GetError

  public boolean getSwitch()
  {//GetSwitch
    return !limitSwitch.get();
  }//GetSwitch

  public void periodic() 
  {//Periodic
    if (pos == null) 
    {//if
        krakenMotor.set(-.1);
        if (!limitSwitch.get()) 
        {//Enclosed if
          krakenMotor.set(0.0);
          krakenMotor.setPosition(0);
          pos = krakenMotor.getPosition();
        }//Enclosed if
    }//if
    Logger.recordOutput("Algae at Home ", !limitSwitch.get());  
  }//Periodic
  
}//END
