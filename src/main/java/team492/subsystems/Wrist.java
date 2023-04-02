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
import TrcCommonLib.trclib.TrcMotorLimitSwitch;
import TrcCommonLib.trclib.TrcPidActuator;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcTriggerDigitalInput;
import TrcFrcLib.frclib.FrcCANCoder;
import TrcFrcLib.frclib.FrcCANFalcon;
import TrcFrcLib.frclib.FrcMotorActuator;
import team492.Robot;
import team492.RobotParams;

public class Wrist
{
    private static final String moduleName = "Wrist";
    private static final String ZERO_CAL_FILE = "wristzero.txt";

    private final Robot robot;
    private final TrcDbgTrace msgTracer;
    private final FrcCANFalcon actuatorMotor;
    private final TrcPidActuator pidActuator;
    private final FrcCANCoder encoder;
    private final TrcTriggerDigitalInput zeroTrigger;
    private boolean encoderSynced = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object so we can access other subsystems.
     * @param msgTracer specifies the tracer to used for message logging, can be null if not provided.
     */
    public Wrist(Robot robot, TrcDbgTrace msgTracer)
    {
        this.robot = robot;
        this.msgTracer = msgTracer;
        TrcPidActuator.Parameters actuatorParams = new TrcPidActuator.Parameters()
            .setScaleAndOffset(RobotParams.WRIST_DEGS_PER_COUNT, RobotParams.WRIST_OFFSET)
            .setPosRange(RobotParams.WRIST_MIN_POS, RobotParams.WRIST_MAX_POS)
            .setPidParams(
                RobotParams.WRIST_KP, RobotParams.WRIST_KI, RobotParams.WRIST_KD, RobotParams.WRIST_KF,
                RobotParams.WRIST_IZONE, RobotParams.WRIST_TOLERANCE)
            .setPowerCompensation(this::getGravityCompensation);

        actuatorMotor = new FrcCANFalcon(moduleName + ".motor", RobotParams.CANID_WRIST);
        actuatorMotor.resetFactoryDefault();
        actuatorMotor.setMotorInverted(RobotParams.WRIST_MOTOR_INVERTED);
        actuatorMotor.enableVoltageCompensation(RobotParams.BATTERY_NOMINAL_VOLTAGE);
        actuatorMotor.setPositionSensorInverted(RobotParams.WRIST_ENCODER_INVERTED);
        actuatorMotor.setFeedbackDevice(FeedbackDevice.IntegratedSensor);
        actuatorMotor.setCurrentLimit(20.0, 40.0, 0.5);

        TrcMotorLimitSwitch lowerLimitSw = new TrcMotorLimitSwitch(moduleName + ".lowerLimitSw", actuatorMotor, false);
        TrcMotorLimitSwitch upperLimitSw = new TrcMotorLimitSwitch(moduleName + ".upperLimitSw", actuatorMotor, true);
        lowerLimitSw.setInverted(RobotParams.WRIST_LOWER_LIMIT_INVERTED);
        upperLimitSw.setInverted(RobotParams.WRIST_UPPER_LIMIT_INVERTED);

        pidActuator = new FrcMotorActuator(
            moduleName, actuatorMotor, lowerLimitSw, upperLimitSw, actuatorParams).getPidActuator();
        pidActuator.setMsgTracer(msgTracer, false);

        double zeroOffset = getZeroPosition(RobotParams.WRIST_ZERO);
        encoder = createCANCoder(
            moduleName + ".encoder", RobotParams.CANID_WRIST_ENCODER, RobotParams.WRIST_ENCODER_INVERTED, zeroOffset);
        actuatorMotor.motor.setSelectedSensorPosition(encoder.getPosition() * RobotParams.WRIST_MOTOR_CPR);
        if (msgTracer != null)
        {
            msgTracer.traceInfo(
                moduleName, "Init: rawEncPos=%.0f, normalizedAbsPos=%.3f",
                encoder.getRawPosition(), encoder.getPosition());
        }

        zeroTrigger = new TrcTriggerDigitalInput(moduleName + ".digitalTrigger", lowerLimitSw);
    }   //Wrist

    /**
     * This method returns the state of the Wrist in a string.
     */
    @Override
    public String toString()
    {
        return String.format(
            Locale.US, "%s: pwr=%.3f, current=%.3f, pos=%.1f/%.1f, Enc=%.0f, AbsEnc=%.0f, LimitSw=%s/%s",
            moduleName, pidActuator.getPower(), actuatorMotor.getMotorCurrent(), pidActuator.getPosition(),
            pidActuator.getPidController().getTarget(), actuatorMotor.motor.getSelectedSensorPosition(),
            encoder.getRawPosition(), pidActuator.isLowerLimitSwitchActive(), pidActuator.isUpperLimitSwitchActive());
    }   //toString

    /**
     * This method creates an encoder for the wrist motor.
     *
     * @param name specifies the instance name of the wrist encoder.
     * @param encoderId specifies the CAN ID of the CANcoder.
     * @param inverted specifies true if the sensor direction should be inverted, false otherwise.
     * @param zeroOffset specifies the zero offset.
     * @return the created wrist encoder.
     */
    private FrcCANCoder createCANCoder(String name, int encoderId, boolean inverted, double zeroOffset)
    {
        final String funcName = "createCANcoder";
        FrcCANCoder canCoder = new FrcCANCoder(name, encoderId);
        ErrorCode errCode;

        // Reset encoder back to factory default to clear potential previous mis-configurations.
        errCode = canCoder.configFactoryDefault(30);
        if (errCode != ErrorCode.OK)
        {
            TrcDbgTrace.globalTraceWarn(
                funcName, "%s: CANcoder.configFactoryDefault failed (code=%s).", name, errCode);
        }

        errCode = canCoder.configFeedbackCoefficient(1.0, "cpr", SensorTimeBase.PerSecond, 30);
        if (errCode != ErrorCode.OK)
        {
            TrcDbgTrace.globalTraceWarn(
                funcName, "%s: CANcoder.configFeedbackCoefficient failed (code=%s).", name, errCode);
        }

        // Configure the encoder to initialize to absolute position value at boot.
        errCode = canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, 30);
        if (errCode != ErrorCode.OK)
        {
            TrcDbgTrace.globalTraceWarn(
                funcName, "%s: CANcoder.configSensorInitializationStrategy failed (code=%s).", name, errCode);
        }

        // Slow down the status frame rate to reduce CAN traffic.
        errCode = canCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100, 30);
        if (errCode != ErrorCode.OK)
        {
            TrcDbgTrace.globalTraceWarn(
                funcName, "%s: CANcoder.setStatusFramePeriod failed (code=%s).", name, errCode);
        }

        // Configure the sensor direction to match the steering motor direction.
        canCoder.setInverted(inverted);
        // Normalize encoder to the range of 0 to 1.0 for a revolution (revolution per count).
        canCoder.setScaleAndOffset(1.0 / RobotParams.CANCODER_CPR, zeroOffset);

        return canCoder;
    }   //createCANCoder

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
     * This method returns the current drawn by the wrist motor.
     *
     * @return wrist motor current drawn.
     */
    public double getCurrent()
    {
        return actuatorMotor.getMotorCurrent();
    }   //getCurrent

    /**
     * This method checks if the motor internal encoder is in sync with the absolute encoder. If not, it will
     * do a re-sync of the motor encoder to the absolute enocder posiition. This method can be called multiple
     * times but it will only perform the re-sync the first time it's called unless forceSync is set to true.
     *
     * @param forceSync specifies true to force performing the encoder resync, false otherwise.
     */
    public void syncEncoder(boolean forceSync)
    {
        final String funcName = "syncEncoder";
        final double encErrThreshold = 20.0;

        if (!encoderSynced || forceSync)
        {
            double absPos = encoder.getPosition() * RobotParams.WRIST_MOTOR_CPR;
            double motorPos = actuatorMotor.getMotorPosition();

            if (Math.abs(absPos - motorPos) > encErrThreshold)
            {
                actuatorMotor.motor.setSelectedSensorPosition(absPos, 0, 0);
                TrcDbgTrace.globalTraceInfo(
                    funcName, "[%.3f] syncEncoder(Before/After)=%.0f/%.0f",
                    TrcTimer.getModeElapsedTime(), motorPos, absPos);
            }
            encoderSynced = true;
        }
    }   //syncEncoder

    /**
     * This method is called to zero calibrate the wrist and save the absolute zero encoder position into a file.
     */
    public void zeroCalibrate()
    {
        zeroTrigger.enableTrigger(this::zeroCalCompletion);
        pidActuator.setPower(RobotParams.WRIST_CAL_POWER);
    }   //zeroCalibrate

    /**
     * This method is call when zero calibration is completed to store the encoder reading to a file.
     *
     * @param context not used.
     */
    private void zeroCalCompletion(Object context)
    {
        final String funcName = "zeroCalCompletion";

        pidActuator.setPower(0.0);
        zeroTrigger.disableTrigger();
        double zeroPos = encoder.getAbsolutePosition();
        saveZeroPosition(zeroPos);
        if (msgTracer != null)
        {
            msgTracer.traceInfo(funcName, "WristZeroCalibrate: zeroPos = %f", zeroPos);
        }
    }   //zeroCalCompletion

    /**
     * This method retrieves the zero calibration data from the calibration data file.
     *
     * @param defZeroPos specifies the default zero position to return if failed to read zero calibration file.
     * @return zero calibration data.
     */
    private double getZeroPosition(double defZeroPos)
    {
        final String funcName = "getZeroPosition";

        try (Scanner in = new Scanner(new FileReader(RobotParams.TEAM_FOLDER + "/" + ZERO_CAL_FILE)))
        {
            double zeroPos = in.nextDouble();
            if (msgTracer != null)
            {
                msgTracer.traceInfo(funcName, "zeroPos=%f", zeroPos);
            }
            return zeroPos;
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
    private void saveZeroPosition(double zeroPos)
    {
        final String funcName = "saveZeroPosition";

        try (PrintStream out = new PrintStream(new FileOutputStream(RobotParams.TEAM_FOLDER + "/" + ZERO_CAL_FILE)))
        {
            out.printf("%f\n", zeroPos);
            TrcDbgTrace.globalTraceInfo(funcName, "Saved zero position: %f!", zeroPos);
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
        return RobotParams.WRIST_MAX_GRAVITY_COMP_POWER *
               Math.sin(Math.toRadians(pidActuator.getPosition() - robot.armPidActuator.getPosition()));
    }   //getGravityCompensation

}   //class Wrist
