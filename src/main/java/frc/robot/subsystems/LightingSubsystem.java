package frc.robot.subsystems;

import org.fairportrobotics.frc.posty.TestableSubsystem;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

//import org.apache.commons.lang3.ObjectUtils.Null;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightingSubsystem extends TestableSubsystem {

    public static CANdle _CANdle;
    String currentColor = "";

    /**
     * Constructs the lighting subsystem with default configuration.
     */
    public LightingSubsystem() 
    {
        super("LightingSubsystem");
       _CANdle = new CANdle(0);
       CANdleConfiguration config = new CANdleConfiguration();
       config.stripType = LEDStripType.BRG;
       _CANdle.configAllSettings(config);
    }

    /**
     * Sets a solid color on the CANdle using rgb and brightness.
     * @param red is an int that represents the additive color of red [0, 255].
     * @param green is an int that represents the additive color of green [0, 255].
     * @param blue is an int that represents the additive color of blue [0, 255].
     * @param brightness is a double that is the bightness by decimal [0, 1] (1 representing max brightness).
     */
    public void setColor(int red, int green, int blue, double brightness) 
    {
        _CANdle.clearAnimation(0);
        _CANdle.setLEDs(red, green, blue);
        _CANdle.configBrightnessScalar(brightness);
    }

    /**
     * Sets the default FireAnimation on the CANdle.
     */
    public void fire()
    {
        FireAnimation animation = new FireAnimation();
        animation.setLedOffset(8);
        animation.setSpeed(0.1);
        animation.setSparking(0.1);
        _CANdle.animate(animation);
    }

    /**
     * Sets the default RainbowAnimation on the CANdle.
     */
    public void rainbow()
    {
        RainbowAnimation animation = new RainbowAnimation();
        animation.setLedOffset(8);
        animation.setSpeed(0.5);
        _CANdle.animate(animation);
    }

    /**
     * Sets the default LarsonAnimation on the CANdle.
     */
    public void larson(){
        LarsonAnimation animation = new LarsonAnimation(255, 0, 0);
        animation.setSpeed(0.01);
        animation.setNumLed(110);
        animation.setBounceMode(BounceMode.Center);
        _CANdle.animate(animation);
    }

    /**
     * Sets the CANdle to the cachow color.
     */
    public void cachow()
    {
        _CANdle.setLEDs(195, 51, 50);
    }

    /**
     * Sets the CANdle to the climb color.
     */
    public void setClimbColor()
    {
        _CANdle.setLEDs(255, 215, 0);
    }

    /**
     * Sets the CANdle to the nemo color.
     */
    public void nemo()
    {
        _CANdle.setLEDs(255, 165, 0);
    }

    /**
     * Sets the CANdle to the underDaWata color.
     */
    public void underDaWata()
    {
        _CANdle.setLEDs(43, 101, 236);
    }
}