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
import java.util.concurrent.atomic.AtomicBoolean;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcTriggerDigitalInput;
import TrcCommonLib.trclib.TrcIntake;
import TrcCommonLib.trclib.TrcTimer;
import TrcFrcLib.frclib.FrcCANSparkMax;
import TrcFrcLib.frclib.FrcDigitalInput;
import team492.Robot;
import team492.RobotParams;

public class Intake
{
    private static final String moduleName = "Intake";

    private final Robot robot;
    private final TrcDbgTrace msgTracer;

    private final FrcCANSparkMax intakeMotor;
    private final FrcDigitalInput intakeSensor;
    private final TrcTriggerDigitalInput intakeTrigger;
    private final TrcIntake intake;

    public Intake(Robot robot, TrcDbgTrace msgTracer, TrcIntake.Parameters params)
    {
        this.robot = robot;
        this.msgTracer = msgTracer;

        intakeMotor = new FrcCANSparkMax(moduleName + ".motor", RobotParams.CANID_INTAKE, true);
        intakeMotor.resetFactoryDefault();
        intakeMotor.setMotorInverted(RobotParams.INTAKE_MOTOR_INVERTED);
        intakeMotor.setBrakeModeEnabled(true);
        intakeMotor.enableVoltageCompensation(RobotParams.BATTERY_NOMINAL_VOLTAGE);
        intakeMotor.setCurrentLimit(20.0, 40.0, 0.5);

        intakeSensor = new FrcDigitalInput(moduleName + ".sensor", RobotParams.DIO_INTAKE_SENSOR);
        intakeTrigger = new TrcTriggerDigitalInput(moduleName + ".trigger", intakeSensor);

        intake = new TrcIntake(moduleName, intakeMotor, params, intakeTrigger, this::intakeTriggerCallback);
    }   //Intake

    /**
     * This method returns the state of the Arm in a string.
     */
    @Override
    public String toString()
    {
        return String.format(
            Locale.US, "%s: pwr=%.1f, hasObject=%s", moduleName, intake.getPower(), intake.hasObject());
    }   //toString

    /**
     * This method returns the TrcIntake object created.
     *
     * @return TrcIntake object.
     */
    public TrcIntake getTrcIntake()
    {
        return intake;
    }   //getTrcIntake

    /**
     * This method is called when the intake sensor is triggered.
     *
     * @param context specifies true if an object is captured, false otherwise.
     */
    private void intakeTriggerCallback(Object context)
    {
        final String funcName = "intakeTriggerCallback";

        boolean sensorActive = ((AtomicBoolean) context).get();
        if (robot.ledIndicator != null)
        {
            // Same issue as TeleOp, forgot to change when I switched
            robot.ledIndicator.setHasObject(sensorActive);
        }

        if (msgTracer != null)
        {
            msgTracer.traceInfo(funcName, "[%.3f] active=%s", TrcTimer.getModeElapsedTime(), sensorActive);
        }
    }   //intakeTriggerCallback

}   //class Intake
