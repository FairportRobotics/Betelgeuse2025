
package frc.robot.subsystems;

import org.fairportrobotics.frc.posty.TestableSubsystem;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.DIOValues;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.SparkMax;

public class AlgaeSubsystem extends TestableSubsystem
{//START

  //VARIABLES
  private TalonFX fenceMotor; //Brings the entire fence like thing down
  private DigitalInput limitSwitch; //idk where tf this is located lmfao
  private SparkMax wheelSpin; //Spins wheel
  private StatusSignal<Angle> pos;
  //VARIABLES

  /**
   * the system that algaes the systemsub
   */
  public AlgaeSubsystem() 
  {//AlgaeSubsystem
    super("AlgaeSubsystem");
    fenceMotor = new TalonFX(17); //Kraken motor
    wheelSpin = new SparkMax(18, null);//Neo motor
    limitSwitch= new DigitalInput(DIOValues.ALGAE_LIMIT_SWITCH);
      //PID LOOP
      TalonFXConfiguration krakenConfig = new TalonFXConfiguration();
      krakenConfig.Slot0.kP = 0.8;
      krakenConfig.Slot0.kI = 0.5;
      krakenConfig.Slot0.kD = 0.3;
      //PID LOOP
      fenceMotor.getConfigurator().apply(krakenConfig);
      pos = fenceMotor.getPosition();
      pos.setUpdateFrequency(50);
      fenceMotor.optimizeBusUtilization();
      fenceMotor.getConfigurator().apply(krakenConfig, 0.050);
    
    registerPOSTTest("Agetator motor is connected", () -> wheelSpin.getBusVoltage() > 0);
    registerPOSTTest("Pivot motor is connected", () -> fenceMotor.isConnected());

  }//ALGAESUBSYSTEM

   /**
   * Spins the wheels, first line
   * brings down the entire fence
   * Calculates and applies voltage to bring it to a specific voltage
   * Note: last line may change position
   */
  public void openBallIntake()
  {//openBallIntake
    wheelSpin.set(.1); 
    fenceMotor.setNeutralMode(NeutralModeValue.Coast);
    PositionVoltage PosRequest = new PositionVoltage(0).withSlot(0);
    fenceMotor.setControl(PosRequest.withPosition(1)); 
  }//BallIntake

  /**
   * @return Position
  */
  public StatusSignal<Angle> getPos()
  {//GetPos
    return pos;
  }//GetPos
  
  /**
   * Stops the wheels from spinning 
   * Sets the kraken motor speed to -.1 so it closes
  */
  public void closeBallIntake()
  {//closeBallIntake
    wheelSpin.stopMotor();
    fenceMotor.set(-.1);
  }//closeBallIntake

  /**
   * No one will understand what this method does, may contain a dead body :Thumbsup:
  */
  public void stopFenceMotor()
  {//stopFenceMotor
    fenceMotor.stopMotor();
  }//stopFenceMotor

  /**
   * @return Error
  */
  public StatusSignal<Double> getError()
  {//getError
    return fenceMotor.getClosedLoopError();
  }//getError

  /**
   * gets the state of the switc
   * @return 
   * True or False state
  */
  public boolean getSwitch()
  {//getSwitch
    return !limitSwitch.get();
  }//getSwitch

  public void periodic() 
  {//Periodic
    /*if (pos == null) 
    {//if
        fenceMotor.set(-.1);
        if (!limitSwitch.get()) 
        {//Enclosed if
          fenceMotor.set(0.0);
          fenceMotor.setPosition(0);
          pos = fenceMotor.getPosition();
        }//Enclosed if
    }//if
    Logger.recordOutput("Algae at Home ", !limitSwitch.get());  
  */
  }//Periodic
  

}//END
