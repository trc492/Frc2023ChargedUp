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
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcExclusiveSubsystem;
import TrcCommonLib.trclib.TrcTimer;
import TrcFrcLib.frclib.FrcCANTalon;
import TrcFrcLib.frclib.FrcDigitalInput;
import TrcFrcLib.frclib.FrcPneumatic;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import team492.RobotParams;

public class Intake implements TrcExclusiveSubsystem
{ 
    private static final String moduleName = "Intake";

    private final LEDIndicator ledIndicator;
    private final TrcDbgTrace msgTracer;
    private final FrcCANTalon intakeLeftMotor;
    private final FrcCANTalon intakeRightMotor;
    private final FrcPneumatic intakePneumatic;
    private final FrcDigitalInput intakeSensor;
    private final TrcTriggerDigitalInput intakeTrigger;
    private boolean sensorActive = false;
    private TrcEvent triggerEvent = null;

    public Intake(LEDIndicator ledIndicator, TrcDbgTrace msgTracer)
    {
        this.ledIndicator = ledIndicator;
        this.msgTracer = msgTracer;

        intakeLeftMotor = new FrcCANTalon(moduleName + ".leftMotor", RobotParams.CANID_INTAKE_LEFT);
        intakeLeftMotor.resetFactoryDefault();
        intakeLeftMotor.setMotorInverted(true);

        intakeRightMotor = new FrcCANTalon(moduleName + ".rightMotor", RobotParams.CANID_INTAKE_RIGHT);
        intakeRightMotor.resetFactoryDefault();
        intakeRightMotor.setMotorInverted(true);

        intakePneumatic = new FrcPneumatic(
            moduleName + ".pneumatic", RobotParams.CANID_PCM, PneumaticsModuleType.REVPH,
            RobotParams.PNEUMATIC_INTAKE_RETRACT, RobotParams.PNEUMATIC_INTAKE_EXTEND);
        intakePneumatic.retract();

        intakeSensor = new FrcDigitalInput("intakeSensor", RobotParams.DIO_INTAKE_SENSOR);
        intakeSensor.setInverted(true);
        intakeTrigger = new TrcTriggerDigitalInput("intakeTrigger", intakeSensor);
        // enableTrigger(triggerEvent);
    }   //Intake

    /**
     * This method returns the state of the Arm in a string.
     */
    @Override
    public String toString()
    {
        return String.format(
            Locale.US, "%s: leftPwr=%.1f, rightPwr=%.1f, Deployed=%s, hasObject=%s",
            moduleName, getLeftMotorPower(), getRightMotorPower(), isExtended(), hasObject());
    }   //toString

    public void cancel(String owner)
    {
        if (validateOwnership(owner))
        {
            intakeLeftMotor.stopMotor();
            intakeRightMotor.stopMotor();
        }
    }   //cancel

    public void cancel()
    {
        cancel(null);
    }   //cancel

    public double getLeftMotorPower()
    {
        return intakeLeftMotor.getMotorPower();
    }   //getLeftMotorPower

    public double getRightMotorPower()
    {
        return intakeRightMotor.getMotorPower();
    }   //getRightMotorPower

    public void setPower(String owner, double delay, double leftPower, double rightPower, double duration)
    {
        final String funcName = "setPower";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(
            funcName, "[%.3f] owner=%s, delay=%.1f, leftPower=%.1f, rightPower=%.1f, duration=%.3f",
            TrcTimer.getModeElapsedTime(), owner, delay, leftPower, rightPower, duration);
        }

        if (validateOwnership(owner))
        {
            intakeLeftMotor.set(delay, leftPower, duration);
            intakeRightMotor.set(delay, rightPower, duration);
        }
    }   //setPower

    public void setPower(double delay, double leftPower, double rightPower, double duration)
    {
        setPower(null, delay, leftPower, rightPower, duration);
    }   //setPower

    public void setPower(double leftPower, double rightPower)
    {
        setPower(null, 0.0, leftPower, rightPower, 0.0);
    }   //setPower

    public void setPower(double power)
    {
        setPower(null, 0.0, power, power, 0.0);
    }   //setPower

    public void extend()
    {
        intakePneumatic.extend();
    }   //extend

    public void extend(double delay)
    {
        intakePneumatic.extend(delay);
    }   //extend

    public void retract()
    {
        intakePneumatic.retract();
    }   //retract

    public void retract(double delay){
        intakePneumatic.retract(delay);
    }

    public boolean isExtended()
    {
        return intakePneumatic.isExtended();
    }

    /**
     * This method checks if the intake sensor is active.
     *
     * @return true if intake sensor is active, false otherwise.
     */
    public boolean hasObject()
    {
        return sensorActive;
    }   //hasObject

    /**
     * This method enables the sensor trigger.
     *
     * @param event specifies the event to signal if the sensor is triggered.
     */
    public void enableTrigger(TrcEvent event)
    {
        triggerEvent = event;
        if (triggerEvent != null)
        {
            triggerEvent.clear();
        }
        intakeTrigger.enableTrigger(this::intakeTriggerCallback);
    }   //enableTrigger

    /**
     * This method disables the sensor trigger.
     */
    public void disableTrigger()
    {
        if (triggerEvent != null)
        {
            triggerEvent.cancel();
            triggerEvent = null;
        }
        intakeTrigger.disableTrigger();
    }   //disableTrigger

    /**
     * This method is called when the intake sensor is triggered.
     *
     * @param context specifies true if an object is captured, false otherwise.
     */
    private void intakeTriggerCallback(Object context)
    {
        final String funcName = "intakeTriggerCallback";
        sensorActive = ((AtomicBoolean) context).get();

        if (triggerEvent != null)
        {
            triggerEvent.signal();
        }

        if (ledIndicator != null)
        {
            ledIndicator.setIntakeHasObject(sensorActive);
        }

        if (msgTracer != null)
        {
            msgTracer.traceInfo(funcName, "[%.3f] active=%s", TrcTimer.getModeElapsedTime(), sensorActive);
        }
    }   //intakeTriggerCallback

}   //class Intake
