/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team492.subsystems;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.PrintStream;
import java.util.Locale;
import java.util.Scanner;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcTriggerDigitalInput;
import TrcCommonLib.trclib.TrcPidActuator;
import TrcFrcLib.frclib.FrcCANTalon;
import TrcFrcLib.frclib.FrcCANTalonLimitSwitch;
import TrcFrcLib.frclib.FrcMotorActuator;
import team492.RobotParams;

public class Arm
{
    private static final String moduleName = "Arm";
    private static final String ZERO_CAL_FILE = "armzero.txt";

    private final TrcDbgTrace msgTracer;
    private final FrcCANTalon actuatorMotor;
    private final TrcPidActuator pidActuator;
    private final TrcTriggerDigitalInput zeroTrigger;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param msgTracer specifies the tracer to used for message logging, can be null if not provided.
     */
    public Arm(TrcDbgTrace msgTracer)
    {
        TrcPidActuator.Parameters actuatorParams = new TrcPidActuator.Parameters()
            .setScaleAndOffset(RobotParams.ARM_DEGS_PER_COUNT, RobotParams.ARM_OFFSET)
            .setPosRange(RobotParams.ARM_MIN_POS, RobotParams.ARM_MAX_POS)
            .setPidParams(
                RobotParams.ARM_KP, RobotParams.ARM_KI, RobotParams.ARM_KD, RobotParams.ARM_KF,
                RobotParams.ARM_IZONE, RobotParams.ARM_TOLERANCE)
            .setPosPresets(RobotParams.ARM_PRESET_TOLERANCE, RobotParams.armPresets)
            .setPowerCompensation(this::getGravityCompensation);

        this.msgTracer = msgTracer;
        actuatorMotor = new FrcCANTalon("ArmMotor", RobotParams.CANID_ARM);
        actuatorMotor.resetFactoryDefault();
        actuatorMotor.setMotorInverted(RobotParams.ARM_MOTOR_INVERTED);
        actuatorMotor.setFeedbackDevice(FeedbackDevice.IntegratedSensor);
        actuatorMotor.setBrakeModeEnabled(true);
        actuatorMotor.enableVoltageCompensation(RobotParams.BATTERY_NOMINAL_VOLTAGE);
        actuatorMotor.setCurrentLimit(20.0, 40.0, 0.5);
        // configMotionMagic(actuatorMotor.motor);

        int zeroOffset = getZeroPosition(RobotParams.ARM_ZERO);
        actuatorMotor.setAbsoluteZeroOffset(0, RobotParams.ARM_ENCODER_CPR - 1, false, zeroOffset);

        FrcCANTalonLimitSwitch lowerLimitSw = new FrcCANTalonLimitSwitch(
            "ArmLowerLimitSw", actuatorMotor, false);
        FrcCANTalonLimitSwitch upperLimitSw = new FrcCANTalonLimitSwitch(
            "ArmUpperLimitSw", actuatorMotor, true);
        lowerLimitSw.setInverted(RobotParams.ARM_LOWER_LIMIT_INVERTED);
        upperLimitSw.setInverted(RobotParams.ARM_UPPER_LIMIT_INVERTED);

        pidActuator = new FrcMotorActuator(
            "Arm", actuatorMotor, lowerLimitSw, upperLimitSw, actuatorParams).getPidActuator();
        pidActuator.setMsgTracer(msgTracer);
        pidActuator.getPidController().setOutputLimit(0.25);

        zeroTrigger = new TrcTriggerDigitalInput(moduleName, lowerLimitSw, this::zeroCalCompletion);
    }   //Arm

    /**
     * This method returns the state of the Arm in a string.
     */
    @Override
    public String toString()
    {
        return String.format(
            Locale.US, "%s: pwr=%.3f, current=%.1f, pos=%.1f/%.1f, Enc=%.0f, LimitSw=%s/%s",
            moduleName, pidActuator.getPower(), actuatorMotor.getMotorCurrent(), pidActuator.getPosition(),
            pidActuator.getPidController().getTarget(), actuatorMotor.motor.getSelectedSensorPosition(),
            pidActuator.isLowerLimitSwitchActive(), pidActuator.isUpperLimitSwitchActive());
    }   //toString

    private void configMotionMagic(TalonSRX motor)
    {
        // Set deadband to super small 0.001 (0.1 %). The default deadband is 0.04 (4 %).
        motor.configNeutralDeadband(0.001, 30);
        // Set relevant frame periods to be at least as fast as periodic rate.
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);
        // Set the peak and nominal outputs.
        motor.configNominalOutputForward(0.0, 30);
        motor.configNominalOutputReverse(0.0, 30);
        motor.configPeakOutputForward(1.0, 30);
        motor.configPeakOutputReverse(-1.0, 30);
        // Set Motion Magic gains in slot0 - see documentation.
        motor.selectProfileSlot(0, 0);
        motor.config_kP(0, 1.0, 30);
        motor.config_kI(0, 0.0, 30);
        motor.config_kD(0, 0.0, 30);
        motor.config_kF(0, 0.05, 30);
        motor.config_IntegralZone(0, 100.0, 30);
        motor.configAllowableClosedloopError(0, 10.0, 30);
        // Set acceleration and vcruise velocity - see documentation.
        motor.configMotionCruiseVelocity(1000.0, 30);
        motor.configMotionAcceleration(1000.0, 30);
    }   //configMotionMagic

    /**
     * This method returns the PidActuator object created.
     *
     * @return PidActuator object.
     */
    public TrcPidActuator getPidActuator()
    {
        return pidActuator;
    }   //getPidActuator

    /**
     * This method is called to zero calibrate the arm and save the absolute zero encoder position into a file.
     */
    public void zeroCalibrate()
    {
        zeroTrigger.setEnabled(true);
    }   //zeroCalibrate

    /**
     * This method is call when zero calibration is completed to store the encoder reading to a file.
     *
     * @param context not used.
     */
    private void zeroCalCompletion(Object context)
    {
        final String funcName = "zeroCalCompletion";
        int zeroPos = actuatorMotor.motor.getSensorCollection().getPulseWidthPosition();

        zeroTrigger.setEnabled(false);
        saveZeroPosition(zeroPos);
        if (msgTracer != null)
        {
            msgTracer.traceInfo(funcName, "ArmZeroCalibrate: zeroPos = %d", zeroPos);
        }
    }   //zeroCalCompletion

    /**
     * This method retrieves the zero calibration data from the calibration data file.
     *
     * @param defZeroPos specifies the default zero position to return if failed to read zero calibration file.
     * @return zero calibration data.
     */
    private int getZeroPosition(int defZeroPos)
    {
        final String funcName = "getZeroPosition";

        try (Scanner in = new Scanner(new FileReader(RobotParams.TEAM_FOLDER + "/" + ZERO_CAL_FILE)))
        {
            return in.nextInt();
        }
        catch (Exception e)
        {
            TrcDbgTrace.globalTraceWarn(funcName, "Zero position file not found, using built-in defaults.");
            return defZeroPos;
        }
    }   //getZeroPosition

    /**
     * This method saves the zero calibration data to the calibration data file.
     *
     * @param zeroPos specifies the zero calibration data to be saved.
     */
    private void saveZeroPosition(int zeroPos)
    {
        final String funcName = "saveZeroPosition";

        try (PrintStream out = new PrintStream(new FileOutputStream(RobotParams.TEAM_FOLDER + "/" + ZERO_CAL_FILE)))
        {
            out.printf("%d\n", zeroPos);
            TrcDbgTrace.globalTraceInfo(funcName, "Saved zero position: %d!", zeroPos);
        }
        catch (FileNotFoundException e)
        {
            e.printStackTrace();
        }
    }   //saveZeroPosition

    /**
     * This method calculates the power required to hold the arm against gravity.
     *
     * @param currPower specifies the current arm power (not used).
     * @return power value to hold arm against gravity.
     */
    public double getGravityCompensation(double currPower)
    {
        return RobotParams.ARM_MAX_GRAVITY_COMP_POWER * Math.sin(Math.toRadians(pidActuator.getPosition()));
    }   //getGravityCompensation

    public double getCurrent()
    {
        return actuatorMotor.getMotorCurrent();
    }

}   //class Arm
