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

package team492;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.PrintStream;
import java.util.Locale;
import java.util.Scanner;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcPidActuator;
import TrcFrcLib.frclib.FrcCANTalon;
import TrcFrcLib.frclib.FrcCANTalonLimitSwitch;
import TrcFrcLib.frclib.FrcMotorActuator;

public class Arm
{
    private static final String moduleName = "Arm";
    private static final String ZERO_CAL_FILE = "armzero.txt";
    private final FrcCANTalon actuatorMotor;
    private final TrcPidActuator pidActuator;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param msgTracer specifies the tracer to used for message logging, can be null if not provided.
     */
    public Arm(TrcDbgTrace msgTracer)
    {
        FrcMotorActuator.MotorParams motorParams = new FrcMotorActuator.MotorParams(
            RobotParams.ARM_MOTOR_INVERTED,
            -1, RobotParams.ARM_LOWER_LIMIT_INVERTED, -1, RobotParams.ARM_UPPER_LIMIT_INVERTED,
            false, RobotParams.BATTERY_NOMINAL_VOLTAGE);
        TrcPidActuator.Parameters actuatorParams = new TrcPidActuator.Parameters()
            .setPosRange(RobotParams.ARM_MIN_POS, RobotParams.ARM_MAX_POS)
            .setScaleOffset(RobotParams.ARM_DEGS_PER_COUNT, RobotParams.ARM_OFFSET)
            .setPidParams(
                RobotParams.ARM_KP, RobotParams.ARM_KI, RobotParams.ARM_KD, RobotParams.ARM_TOLERANCE)
            .setZeroCalibratePower(RobotParams.ARM_CAL_POWER);
        actuatorMotor = new FrcCANTalon("ArmMotor", RobotParams.CANID_ARM);

        if (motorParams.batteryNominalVoltage > 0.0)
        {
            actuatorMotor.enableVoltageCompensation(RobotParams.BATTERY_NOMINAL_VOLTAGE);
        }

        int zeroOffset = getArmZeroPosition();
        actuatorMotor.setAbsoluteZeroOffset(0, RobotParams.ARM_ENCODER_CPR - 1, false, zeroOffset);

        FrcCANTalonLimitSwitch lowerLimitSw = new FrcCANTalonLimitSwitch(
            "ArmLowerLimitSw", actuatorMotor, false);
        FrcCANTalonLimitSwitch upperLimitSw = new FrcCANTalonLimitSwitch(
            "ArmUpperLimitSw", actuatorMotor, true);
        lowerLimitSw.setInverted(motorParams.lowerLimitSwitchInverted);
        upperLimitSw.setInverted(motorParams.upperLimitSwitchInverted);

        pidActuator = new FrcMotorActuator(
            "Arm", actuatorMotor, lowerLimitSw, upperLimitSw, motorParams, actuatorParams).getPidActuator();
        pidActuator.setMsgTracer(msgTracer);
    }   //Arm

    /**
     * This method returns the state of the Arm in a string.
     */
    @Override
    public String toString()
    {
        return String.format(
            Locale.US, "%s: pwr=%.1f, pos=%.1f, LimitSw=%s/%s",
            moduleName, pidActuator.getPower(), pidActuator.getPosition(), pidActuator.isLowerLimitSwitchActive(),
            pidActuator.isUpperLimitSwitchActive());
    }   //toString

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
        pidActuator.zeroCalibrate(this::zeroCalCompletion, null);
    }   //zeroCalibrate

    /**
     * This method is call when zero calibration is completed to store the encoder reading to a file.
     *
     * @param context not used.
     */
    private void zeroCalCompletion(Object context)
    {
        saveArmZeroPosition(actuatorMotor.motor.getSensorCollection().getPulseWidthPosition());
    }   //zeroCalCompletion

    /**
     * This method retrieves the arm zero calibration data from the calibration data file.
     *
     * @return zero calibration data of the arm.
     */
    private int getArmZeroPosition()
    {
        final String funcName = "getArmZeroPosition";

        try (Scanner in = new Scanner(new FileReader(RobotParams.TEAM_FOLDER + "/" + ZERO_CAL_FILE)))
        {
            return in.nextInt();
        }
        catch (Exception e)
        {
            TrcDbgTrace.globalTraceWarn(funcName, "Arm zero position file not found, using built-in defaults.");
            return RobotParams.ARM_ZERO;
        }
    }   //getArmZeroPosition

    /**
     * This method saves the zero calibration data to the calibration data file.
     *
     * @param armZero specifies the zero calibration data to be saved.
     */
    private void saveArmZeroPosition(int armZero)
    {
        final String funcName = "saveArmZeroPosition";

        try (PrintStream out = new PrintStream(new FileOutputStream(RobotParams.TEAM_FOLDER + "/" + ZERO_CAL_FILE)))
        {
            out.printf("%d\n", armZero);
            TrcDbgTrace.globalTraceInfo(funcName, "Saved arm zero: %d!", armZero);
        }
        catch (FileNotFoundException e)
        {
            e.printStackTrace();
        }
    }   //saveArmZeroPosition

}   //class Arm
