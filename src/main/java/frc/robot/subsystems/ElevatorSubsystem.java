package frc.robot.subsystems;//lol "package" HA AH AHAHAHAGGGG *Cough noise *Cough noise*5 *Falls down stairs... - Lukas

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorLevels;
import frc.robot.commands.ElevatorGoToLevelCommand;
import frc.robot.commands.ElevatorGoToLevelCommand.EncoderGetter;

public class ElevatorSubsystem extends SubsystemBase {
    /**
     * Makes the limit switch getter value more intuitive because it should be true
     * when its pressed, but it actually returns false when its pressed.
     */
    public class LimitSwitchFixer {
        private DigitalInput limitSwitch;

        public LimitSwitchFixer(DigitalInput limitSwitch) {
            this.limitSwitch = limitSwitch;
        }

        public boolean get() {
            return !limitSwitch.get();
        }
    }

    // The home positions of the elevator motors, initially we don't know the home
    // positions of the elevator.
    private double leftHomePos = Double.MAX_VALUE, rightHomePos = Double.MAX_VALUE;

    // The motors of the elevator.
    private final TalonFX elevatorLeftMotor = applyDefaultSettings(new TalonFX(Constants.ElevatorMotors.LEFT_ID), true),
            elevatorRightMotor = applyDefaultSettings(new TalonFX(Constants.ElevatorMotors.RIGHT_ID), false);

    // Stores the position PID that does the motor control.
    private final PositionVoltage LEFT_POS_VOLTAGE = new PositionVoltage(0).withSlot(0),
            RIGHT_POS_VOLTAGE = new PositionVoltage(0).withSlot(0);

    // Information suppliers
    private final LimitSwitchFixer bottomLimitSwitch = new LimitSwitchFixer(
            new DigitalInput(Constants.DIOValues.ELEVATORLIMIT));
    private final StatusSignal<Angle> leftPos = elevatorLeftMotor.getPosition(),
            rightPos = elevatorRightMotor.getPosition();

    // The level that the elevator should go to.
    private volatile ElevatorLevels goToLevel = Constants.ElevatorLevels.HOME;

    // Stores the encoder getter for the elevator for faster access.
    private EncoderGetter encoderGetter = ElevatorGoToLevelCommand.ENCODER_GETTER;

    // Logic variables for the periodic method.
    private boolean isBraked = false;

    /**
     * Applies default settings to the motor.
     * 
     * @param motor                    is the motor to apply the settings to.
     * @param counterClockwisePositive is true if the motor is counter clockwise
     *                                 positive, false for clockwise positive.
     * @return the motor with the default settings applied.
     */
    private static TalonFX applyDefaultSettings(TalonFX motor, boolean counterClockwisePositive) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = .7;
        config.Slot0.kI = .5;
        config.Slot0.kD = .1;
        if (counterClockwisePositive)
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        else
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor.getConfigurator().apply(config);
        motor.getPosition().setUpdateFrequency(50);
        motor.optimizeBusUtilization();
        return motor;
    }

    /**
     * Gets the level of the elevator.
     */
    public ElevatorLevels getGoToLevel() {
        return goToLevel;
    }

    /**
     * Sets the level of the elevator.
     * 
     * @param newLevel is the level to set the elevator to.
     *                 If the new level is null, the elevator is already at the
     *                 level, or the elevator is not initialized, this method does
     *                 nothing.
     */
    public void setLevel(ElevatorLevels newLevel) {
        goToLevel = newLevel;
    }

    /**
     * (This the initializer of the elevator subsystem when the robot is enabled)
     * Moves the elevator down while the leftHomePos or rightHomePos are not set.
     * When the bottom limit switch is pressed, leftHomePos and rightHomePos are
     * initialized and the motors are stopped.
     */
    @Override
    public void periodic() {
        if (isAtTheGoToLevel())
            atLevelChecks();
        else
            notAtLevelChecks();
    }

    /**
     * Checks when the elevator is at the correct spot.
     */
    private void atLevelChecks() {
        /**
         * If the elevator is not braked (is moving) and the elevator is at the correct
         * level, stop the motors.
         */
        if (!isBraked) {
            stopMotors();
            if (ElevatorLevels.HOME.equals(goToLevel))
                setHomePositions(leftPos.refresh().getValueAsDouble(), rightPos.refresh().getValueAsDouble());
            return;
        }
    }

    /**
     * Checks when the elevator is not at the correct spot.
     */
    private void notAtLevelChecks() {
        /**
         * If not initialied, move down.
         */
        if (notInitialized()) {
            moveDown();
            return;
        }

        /**
         * If the elevator is braked (is not moving), the goToLevel is not HOME, move
         * the elevator.
         */
        if (isBraked && !ElevatorLevels.HOME.equals(goToLevel)) {
            moveElevator();
            return;
        }

        /**
         * If the goToLevel of the elevator is HOME (reguardless of whether the elevator
         * is braked or not), move down.
         */
        if (ElevatorLevels.HOME.equals(goToLevel)) {
            moveDown();
            return;
        }
    }

    /**
     * Checks if the elevator is not initialized
     * 
     * @return true if the elevator is not initialized, false if the elevator is
     *         initialized.
     */
    private boolean notInitialized() {
        return leftHomePos == Double.MAX_VALUE || rightHomePos == Double.MAX_VALUE;
    }

    /**
     * If either its going home at the bottom limit switch is pressed or it is
     * initialized, its going somewhere else, and its distance from the target level
     * is less than 0.1, return true, otherwise return false.
     * 
     * @return true if the elevator is at the correct spot, false otherwise.
     */
    private boolean isAtTheGoToLevel() {
        return (ElevatorLevels.HOME.equals(goToLevel) && bottomLimitSwitch.get()) || (!notInitialized()
                && !ElevatorLevels.HOME.equals(goToLevel)
                && Math.abs(leftPos.refresh().getValueAsDouble() + leftHomePos - encoderGetter.get(goToLevel)) <= 0.1);
    }

    /**
     * Moves the elevator down.
     */
    public void moveDown() {
        goToLevel = ElevatorLevels.HOME;
        double speed;
        if (notInitialized())
            speed = -0.05;
        else
            speed = Math.min(-0.175 * (((double) leftPos.refresh().getValueAsDouble() + leftHomePos)
                    / encoderGetter.get(ElevatorLevels.values()[ElevatorLevels.values().length - 1])), -0.05);
        setMotorNeutralMode(NeutralModeValue.Coast);
        elevatorLeftMotor.set(speed);
        elevatorRightMotor.set(speed);
    }

    /**
     * Gets the bottom limit switch of the elevator as a boolean.
     * 
     * @return true if the bottom limit switch is pressed, false otherwise.
     */
    public boolean getbottomLimitSwitchAsBoolean() {
        return bottomLimitSwitch.get();
    }

    /**
     * Sets the home positions of the elevator motors.
     * 
     * @param left  is the home position of the left motor.
     * @param right is the home position of the right motor.
     */
    private void setHomePositions(double left, double right) {
        leftHomePos = left;
        rightHomePos = right;
    }

    /**
     * Moves the elevator to the level specified by goToLevel.
     */
    private void moveElevator() {
        setMotorNeutralMode(NeutralModeValue.Coast);
        if (ElevatorLevels.HOME.equals(goToLevel))
            moveDown();
        else
            setMotorPositions(encoderGetter.get(goToLevel));
    }

    /**
     * Stops the elevator motors and sets the motors to brake.
     */
    public void stopMotors() {
        setMotorNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * Sets the neutral mode of both elevator motors.
     * Updates the isBraked variable accordingly.
     * 
     * @param modeValue is the neutral mode of the motors.
     */
    private void setMotorNeutralMode(NeutralModeValue modeValue) {
        if (NeutralModeValue.Brake.equals(modeValue)) {
            elevatorLeftMotor.stopMotor();
            elevatorRightMotor.stopMotor();
            isBraked = true;
        } else
            isBraked = false;
        elevatorLeftMotor.setNeutralMode(modeValue);
        elevatorRightMotor.setNeutralMode(modeValue);
    }

    /**
     * Sets the positions of both elevator motors.
     * 
     * @param position is the position to set the motors to.
     */
    private void setMotorPositions(double position) {
        elevatorLeftMotor.setControl(LEFT_POS_VOLTAGE.withPosition(leftHomePos + position));
        elevatorRightMotor.setControl(RIGHT_POS_VOLTAGE.withPosition(rightHomePos + position));
    }

    /**
     * Checks if the elevator is fininshed moving.
     * 
     * @return true if the elevator is not moving, false otherwise.
     */
    public boolean isFinishedMoving() {
        return isBraked;
    }
}