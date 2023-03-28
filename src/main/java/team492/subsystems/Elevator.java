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

import java.util.Locale;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcMotorLimitSwitch;
import TrcCommonLib.trclib.TrcPidActuator;
import TrcFrcLib.frclib.FrcCANSparkMax;
import TrcFrcLib.frclib.FrcMotorActuator;
import team492.RobotParams;

public class Elevator
{
    private static final String moduleName = "Elevator";
    // private static final String ZERO_CAL_FILE = "elevatorzero.txt";

    private final TrcDbgTrace msgTracer;
    private final FrcCANSparkMax actuatorMotor;
    private final TrcPidActuator pidActuator;
    private double autoStartOffset = 0.0;
    // private final TrcDigitalInputTrigger zeroTrigger;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param msgTracer specifies the tracer to used for message logging, can be null if not provided.
     */
    public Elevator(TrcDbgTrace msgTracer)
    {
        TrcPidActuator.Parameters actuatorParams = new TrcPidActuator.Parameters()
            .setScaleAndOffset(RobotParams.ELEVATOR_INCHES_PER_COUNT, RobotParams.ELEVATOR_OFFSET)
            .setPosRange(RobotParams.ELEVATOR_MIN_POS, RobotParams.ELEVATOR_MAX_POS)
            .setPidParams(
                RobotParams.ELEVATOR_KP, RobotParams.ELEVATOR_KI, RobotParams.ELEVATOR_KD, RobotParams.ELEVATOR_KF,
                RobotParams.ELEVATOR_IZONE, RobotParams.ELEVATOR_TOLERANCE)
            .setPosPresets(RobotParams.ELEVATOR_PRESET_TOLERANCE, RobotParams.elevatorPresets)
            .setZeroCalibratePower(RobotParams.ELEVATOR_CAL_POWER)
            .resetPositionOnLowerLimit(false)
            .setStallProtectionParams(
                RobotParams.ELEVATOR_STALL_MIN_POWER, RobotParams.ELEVATOR_STALL_TOLERANCE,
                RobotParams.ELEVATOR_STALL_TIMEOUT, RobotParams.ELEVATOR_RESET_TIMEOUT);


        this.msgTracer = msgTracer;
        actuatorMotor = new FrcCANSparkMax("ElevatorMotor", RobotParams.CANID_ELEVATOR, true);
        actuatorMotor.resetFactoryDefault();
        actuatorMotor.setMotorInverted(RobotParams.ELEVATOR_MOTOR_INVERTED);
        actuatorMotor.setBrakeModeEnabled(false);
        actuatorMotor.enableVoltageCompensation(RobotParams.BATTERY_NOMINAL_VOLTAGE);

        // int zeroOffset = getZeroPosition(RobotParams.ELEVATOR_ZERO);
        // actuatorMotor.setAbsoluteZeroOffset(0, RobotParams.NEO_CPR - 1, false, zeroOffset);

        TrcMotorLimitSwitch lowerLimitSw = new TrcMotorLimitSwitch("ElevatorLowerLimitSw", actuatorMotor, false);
        TrcMotorLimitSwitch upperLimitSw = new TrcMotorLimitSwitch("ElevatorUpperLimitSw", actuatorMotor, true);
        lowerLimitSw.setInverted(RobotParams.ELEVATOR_LOWER_LIMIT_INVERTED);
        upperLimitSw.setInverted(RobotParams.ELEVATOR_UPPER_LIMIT_INVERTED);

        pidActuator = new FrcMotorActuator(
            "Elevator", actuatorMotor, lowerLimitSw, upperLimitSw, actuatorParams).getPidActuator();
        pidActuator.setMsgTracer(msgTracer);

        // zeroTrigger = new TrcDigitalInputTrigger(moduleName, lowerLimitSw, this::zeroCalCompletion);
    }   //Elevator

    /**
     * This method returns the state of the Elevator in a string.
     */
    @Override
    public String toString()
    {
        return String.format(
            Locale.US, "%s: pwr=%.3f, current=%.3f, pos=%.1f/%.1f, Enc=%.0f, LimitSw=%s/%s",
            moduleName, pidActuator.getPower(), actuatorMotor.getMotorCurrent(), getPosition(),
            pidActuator.getPidController().getTarget(), actuatorMotor.motor.getEncoder().getPosition(),
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

    /**
     * This method sets the elevator position offset at the start of autonomous.
     */
    public void setAutoStartOffset(double offset)
    {
        final String funcName = "setAutoStartOffset";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(funcName, "offset=%.3f", offset);
        }

        pidActuator.getMotor().resetMotorPosition();
        autoStartOffset = offset;
    }   //setAutoStartOffset

    /**
     * This method returns the elevator position. When a match is started in autonomous, the arm is tucked inside
     * the robot and we can't zero calibrate it and the elevator is slightly up to accommodate the arm. Therefore,
     * we will compensate this height. Once the elevator is calibrated, this compensation will be cleared.
     *
     * @return elevator position in inches extension.
     */
    public double getPosition()
    {
        return pidActuator.getPosition() + autoStartOffset;
    }   //getPosition

    /**
     * This method starts the zero calibration process.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     */
    public void zeroCalibrate(String owner)
    {
        autoStartOffset = 0.0;
        pidActuator.zeroCalibrate(owner);
    }   //zeroCalibrate

    // /**
    //  * This method is called to zero calibrate the elevator and save the absolute zero encoder position into a file.
    //  */
    // public void zeroCalibrate()
    // {
    //     zeroTrigger.setEnabled(true);
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

    //     zeroTrigger.setEnabled(false);
    //     saveZeroPosition(zeroPos);
    //     if (msgTracer != null)
    //     {
    //         msgTracer.traceInfo(funcName, "ElevatorZeroCalibrate: zeroPos = %d", zeroPos);
    //     }
    // }   //zeroCalCompletion

    // /**
    //  * This method retrieves the zero calibration data from the calibration data file.
    //  *
    //  * @param defZeroPos specifies the default zero position to return if failed to read zero calibration file.
    //  * @return zero calibration data.
    //  */
    // private int getZeroPosition(int defZeroPos)
    // {
    //     final String funcName = "getZeroPosition";

    //     try (Scanner in = new Scanner(new FileReader(RobotParams.TEAM_FOLDER + "/" + ZERO_CAL_FILE)))
    //     {
    //         return in.nextInt();
    //     }
    //     catch (Exception e)
    //     {
    //         TrcDbgTrace.globalTraceWarn(funcName, "Zero position file not found, using built-in defaults.");
    //         return defZeroPos;
    //     }
    // }   //getZeroPosition

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

}   //class Elevator
