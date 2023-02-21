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
import TrcCommonLib.trclib.TrcPidActuator;
import TrcFrcLib.frclib.FrcCANSparkMax;
import TrcFrcLib.frclib.FrcCANSparkMaxLimitSwitch;
import TrcFrcLib.frclib.FrcMotorActuator;
import team492.RobotParams;

public class Elevator
{
    private static final String moduleName = "Elevator";
    private final TrcPidActuator pidActuator;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param msgTracer specifies the tracer to used for message logging, can be null if not provided.
     */
    public Elevator(TrcDbgTrace msgTracer)
    {
        FrcMotorActuator.MotorParams motorParams = new FrcMotorActuator.MotorParams(
            RobotParams.ELEVATOR_MOTOR_INVERTED,
            -1, RobotParams.ELEVATOR_LOWER_LIMIT_INVERTED, -1, RobotParams.ELEVATOR_UPPER_LIMIT_INVERTED,
            false, RobotParams.BATTERY_NOMINAL_VOLTAGE);
        TrcPidActuator.Parameters actuatorParams = new TrcPidActuator.Parameters()
            .setPosRange(RobotParams.ELEVATOR_MIN_POS, RobotParams.ELEVATOR_MAX_POS)
            .setScaleOffset(RobotParams.ELEVATOR_INCHES_PER_COUNT, RobotParams.ELEVATOR_OFFSET)
            .setPidParams(
                RobotParams.ELEVATOR_KP, RobotParams.ELEVATOR_KI, RobotParams.ELEVATOR_KD,
                RobotParams.ELEVATOR_TOLERANCE)
            .setZeroCalibratePower(RobotParams.ELEVATOR_CAL_POWER);
        FrcCANSparkMax actuatorMotor = new FrcCANSparkMax("ElevatorMotor", RobotParams.CANID_ELEVATOR, true);

        if (motorParams.batteryNominalVoltage > 0.0)
        {
            actuatorMotor.enableVoltageCompensation(motorParams.batteryNominalVoltage);
        }

        FrcCANSparkMaxLimitSwitch lowerLimitSw = new FrcCANSparkMaxLimitSwitch(
            "ElevatorLowerLimitSw", actuatorMotor, false);
        FrcCANSparkMaxLimitSwitch upperLimitSw = new FrcCANSparkMaxLimitSwitch(
            "ElevatorUpperLimitSw", actuatorMotor, true);
        lowerLimitSw.setInverted(motorParams.lowerLimitSwitchInverted);
        upperLimitSw.setInverted(motorParams.upperLimitSwitchInverted);

        pidActuator = new FrcMotorActuator(
            "Elevator", actuatorMotor, lowerLimitSw, upperLimitSw, motorParams, actuatorParams).getPidActuator();
        pidActuator.setMsgTracer(msgTracer);
    }   //Elevator

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

}   //class Elevator
