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

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcTriggerDigitalInput;
import TrcCommonLib.trclib.TrcPidActuator;
import TrcFrcLib.frclib.FrcAnalogEncoder;
import TrcFrcLib.frclib.FrcCANCoder;
import TrcFrcLib.frclib.FrcCANFalcon;
import TrcFrcLib.frclib.FrcCANTalonLimitSwitch;
import TrcFrcLib.frclib.FrcDigitalInput;
import TrcFrcLib.frclib.FrcMotorActuator;
import edu.wpi.first.wpilibj.CAN;
import team492.RobotParams;

public class Wrist
{
    private static final String moduleName = "Wrist";
    // TODO: TBD
    private static final String ZERO_CAL_FILE = "wristzero.txt";

    private final TrcDbgTrace msgTracer;
    private final FrcCANFalcon actuatorMotor;
    private final FrcCANCoder encoder;
    private final TrcPidActuator pidActuator;
    // private final TrcTriggerDigitalInput zeroTrigger;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param msgTracer specifies the tracer to used for message logging, can be null if not provided.
     */
    public Wrist(TrcDbgTrace msgTracer)
    {
        TrcPidActuator.Parameters actuatorParams = new TrcPidActuator.Parameters()
            .setScaleAndOffset(RobotParams.WRIST_DEGS_PER_COUNT, RobotParams.WRIST_OFFSET)
            .setPosRange(RobotParams.WRIST_MIN_POS, RobotParams.WRIST_MAX_POS)
            .setPidParams(
                RobotParams.WRIST_KP, RobotParams.WRIST_KI, RobotParams.WRIST_KD, RobotParams.WRIST_KF,
                RobotParams.WRIST_IZONE, RobotParams.WRIST_TOLERANCE);

        this.msgTracer = msgTracer;
        actuatorMotor = new FrcCANFalcon("WristMotor", RobotParams.CANID_WRIST);
        actuatorMotor.resetFactoryDefault();
        actuatorMotor.setMotorInverted(RobotParams.WRIST_MOTOR_INVERTED);
        actuatorMotor.setPositionSensorInverted(RobotParams.WRIST_ENCODER_INVERTED);
        actuatorMotor.setFeedbackDevice(FeedbackDevice.IntegratedSensor);
        actuatorMotor.setBrakeModeEnabled(true);
        actuatorMotor.enableVoltageCompensation(RobotParams.BATTERY_NOMINAL_VOLTAGE);
        actuatorMotor.setCurrentLimit(20.0, 40.0, 0.5);

        encoder = new FrcCANCoder(moduleName + ".encoder", RobotParams.CANID_WRIST_ENCODER);
        ErrorCode errCode;
        // Reset encoder back to factory default to clear potential previous mis-configurations.
        errCode = encoder.configFactoryDefault(10);
        if (errCode != ErrorCode.OK)
        {
            msgTracer.traceWarn(
                moduleName, "CANcoder.configFactoryDefault failed (code=%s).", errCode);
        }
        errCode = encoder.configFeedbackCoefficient(1.0, "cpr", SensorTimeBase.PerSecond, 10);
        if (errCode != ErrorCode.OK)
        {
            msgTracer.traceWarn(
                moduleName, "CANcoder.configFeedbackCoefficient failed (code=%s).", errCode);
        }
        // Configure the encoder to initialize to absolute position value at boot.
        errCode = encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, 10);
        if (errCode != ErrorCode.OK)
        {
            msgTracer.traceWarn(
                moduleName, "%s: CANcoder.configSensorInitializationStrategy failed (code=%s).",errCode);
        }
        // Slow down the status frame rate to reduce CAN traffic.
        errCode = encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100, 10);
        if (errCode != ErrorCode.OK)
        {
            msgTracer.traceWarn(
                moduleName, "%s: CANcoder.setStatusFramePeriod failed (code=%s).", errCode);
        }
        // Configure the sensor direction to match the steering motor direction.
        encoder.setInverted(RobotParams.WRIST_ENCODER_INVERTED);
        // Normalize encoder to the range of 0 to 1.0 for a revolution (revolution per count).
        int zeroOffset = getZeroPosition(RobotParams.WRIST_ZERO);
        encoder.setScaleAndOffset(1.0 / RobotParams.CANCODER_CPR, zeroOffset);

        FrcCANFalconLimitSwitch lowerLimitSw = new FrcCANFalconLimitSwitch(
            "WristLowerLimitSw", actuatorMotor, false);
        FrcCANFalconLimitSwitch upperLimitSw = new FrcCANFalconLimitSwitch(
            "WristUpperLimitSw", actuatorMotor, true);
        lowerLimitSw.setInverted(RobotParams.WRIST_LOWER_LIMIT_INVERTED);
        upperLimitSw.setInverted(RobotParams.WRIST_UPPER_LIMIT_INVERTED);

        pidActuator = new FrcMotorActuator(
            "Wrist", actuatorMotor, lowerLimitSw, upperLimitSw, actuatorParams).getPidActuator();
        pidActuator.setMsgTracer(msgTracer, false);

        // zeroTrigger = new TrcTriggerDigitalInput(moduleName, lowerLimitSw);
    }   //Wrist

    /**
     * This method returns the state of the Wrist in a string.
     */
    @Override
    public String toString()
    {
        return String.format(
            Locale.US, "%s: pwr=%.3f, current=%.3f, pos=%.1f/%.1f, Enc=%.0f, LimitSw=%s/%s",
            moduleName, pidActuator.getPower(), actuatorMotor.getMotorCurrent(), pidActuator.getPosition(),
            pidActuator.getPidController().getTarget(), actuatorMotor.motor.getSelectedSensorPosition(),
            pidActuator.isLowerLimitSwitchActive(), pidActuator.isUpperLimitSwitchActive());
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

    // /**
    //  * This method is called to zero calibrate the wrist and save the absolute zero encoder position into a file.
    //  */
    // public void zeroCalibrate()
    // {
    //     zeroTrigger.enableTrigger(this::zeroCalCompletion);
    //     pidActuator.setPower(RobotParams.WRIST_CAL_POWER);
    // }   //zeroCalibrate

    // /**
    //  * This method is call when zero calibration is completed to store the encoder reading to a file.
    //  *
    //  * @param context not used.
    //  */
    // private void zeroCalCompletion(Object context)
    // {
    //     final String funcName = "zeroCalCompletion";
    //     int zeroPos = actuatorMotor.motor.getSensorCollection().getPulseWidthPosition();

    //     zeroTrigger.disableTrigger();
    //     saveZeroPosition(zeroPos);
    //     if (msgTracer != null)
    //     {
    //         msgTracer.traceInfo(funcName, "WristZeroCalibrate: zeroPos = %d", zeroPos);
    //     }
    // }   //zeroCalCompletion

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

    // /**
    //  * This method saves the zero calibration data to the calibration data file.
    //  *
    //  * @param zeroPos specifies the zero calibration data to be saved.
    //  */
    // private void saveZeroPosition(int zeroPos)
    // {
    //     final String funcName = "saveZeroPosition";

    //     try (PrintStream out = new PrintStream(new FileOutputStream(RobotParams.TEAM_FOLDER + "/" + ZERO_CAL_FILE)))
    //     {
    //         out.printf("%d\n", zeroPos);
    //         TrcDbgTrace.globalTraceInfo(funcName, "Saved zero position: %d!", zeroPos);
    //     }
    //     catch (FileNotFoundException e)
    //     {
    //         e.printStackTrace();
    //     }
    // }   //saveZeroPosition

    /**
     * This method returns the current drawn by the wrist motor.
     *
     * @return wrist motor current drawn.
     */
    public double getCurrent()
    {
        return actuatorMotor.getMotorCurrent();
    }

}   //class Wrist