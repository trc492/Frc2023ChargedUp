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
import team492.Robot;
import team492.RobotParams;

public class WeedWhacker implements TrcExclusiveSubsystem
{ 
    private static final String moduleName = "WeedWhacker";

    private final Robot robot;
    private final TrcDbgTrace msgTracer;
    private final FrcCANTalon weedWhackerLeftMotor;
    private final FrcCANTalon weedWhackerRightMotor;
    private final FrcPneumatic weedWhackerPneumatic;
    private final FrcDigitalInput weedWhackerSensor;
    private final TrcTriggerDigitalInput weedWhackerTrigger;
    private TrcEvent triggerEvent = null;
    private boolean sensorActive = false;

    public WeedWhacker(Robot robot, TrcDbgTrace msgTracer)
    {
        this.robot = robot;
        this.msgTracer = msgTracer;

        weedWhackerLeftMotor = new FrcCANTalon(moduleName + ".leftMotor", RobotParams.CANID_WEEDWHACKER_LEFT_MOTOR);
        weedWhackerLeftMotor.resetFactoryDefault();
        weedWhackerLeftMotor.setMotorInverted(false);

        weedWhackerRightMotor = new FrcCANTalon(moduleName + ".rightMotor", RobotParams.CANID_WEEDWHACKER_RIGHT_MOTOR);
        weedWhackerRightMotor.resetFactoryDefault();
        weedWhackerRightMotor.setMotorInverted(false);

        weedWhackerPneumatic = new FrcPneumatic(
            moduleName + ".pneumatic", RobotParams.CANID_PCM, PneumaticsModuleType.REVPH,
            RobotParams.PNEUMATIC_WEEDWHACKER_RETRACT, RobotParams.PNEUMATIC_WEEDWHACKER_EXTEND);
        weedWhackerPneumatic.retract();

        weedWhackerSensor = new FrcDigitalInput(moduleName + ".sensor", RobotParams.DIO_WEEDWHACKER_SENSOR);
        weedWhackerSensor.setInverted(true);
        weedWhackerTrigger = new TrcTriggerDigitalInput(moduleName + ".trigger", weedWhackerSensor);
        enableTrigger(null);
    }   //WeedWhacker

    /**
     * This method returns the state of the Arm in a string.
     */
    @Override
    public String toString()
    {
        return String.format(
            Locale.US, "%s: leftPwr=%.1f, rightPwr=%.1f, Extended=%s, hasObject=%s",
            moduleName, getLeftMotorPower(), getRightMotorPower(), isExtended(), hasObject());
    }   //toString

    public void cancel(String owner)
    {
        if (validateOwnership(owner))
        {
            weedWhackerLeftMotor.stopMotor();
            weedWhackerRightMotor.stopMotor();
        }
    }   //cancel

    public void cancel()
    {
        cancel(null);
    }   //cancel

    public double getLeftMotorPower()
    {
        return weedWhackerLeftMotor.getMotorPower();
    }   //getLeftMotorPower

    public double getRightMotorPower()
    {
        return weedWhackerRightMotor.getMotorPower();
    }   //getRightMotorPower

    public void setPower(
        String owner, double delay, double leftPower, double rightPower, double duration, TrcEvent event)
    {
        final String funcName = "setPower";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(
            funcName, "[%.3f] owner=%s, delay=%.1f, leftPower=%.1f, rightPower=%.1f, duration=%.3f, event=%s",
            TrcTimer.getModeElapsedTime(), owner, delay, leftPower, rightPower, duration, event);
        }

        if (validateOwnership(owner))
        {
            weedWhackerLeftMotor.set(delay, leftPower, duration, event);
            weedWhackerRightMotor.set(delay, rightPower, duration, event);
        }
    }   //setPower

    public void setPower(String owner, double delay, double power, double duration, TrcEvent event)
    {
       setPower(owner, delay, power, power, duration, event);
    }  //setPower

    public void setPower(double delay, double leftPower, double rightPower, double duration, TrcEvent event)
    {
        setPower(null, delay, leftPower, rightPower, duration, event);
    }   //setPower

    public void setPower(double delay, double leftPower, double rightPower, double duration)
    {
        setPower(null, delay, leftPower, rightPower, duration, null);
    }   //setPower

    public void setPower(double leftPower, double rightPower)
    {
        setPower(null, 0.0, leftPower, rightPower, 0.0, null);
    }   //setPower

    public void setPower(double power)
    {
        setPower(null, 0.0, power, power, 0.0, null);
    }   //setPower

    public void extend(double delay)
    {
        weedWhackerPneumatic.extend(delay);
        if (robot.armPidActuator != null)
        {
            robot.armPidActuator.setPositionRange(RobotParams.ARM_MIN_POS_WEEDWHACKER_DOWN, RobotParams.ARM_MAX_POS);
        }
    }   //extend

    public void extend()
    {
        extend(0.0);
    }   //extend

    public void retract(double delay)
    {
        weedWhackerPneumatic.retract(delay);
        if (robot.armPidActuator != null)
        {
            robot.armPidActuator.setPositionRange(RobotParams.ARM_MIN_POS_WEEDWHACKER_UP, RobotParams.ARM_MAX_POS);
        }
    }   //retract

    public void retract()
    {
        retract(0.0);
    }   //retract

    public boolean isExtended()
    {
        return weedWhackerPneumatic.isExtended();
    }   //isExtended

    /**
     * This method checks if the weedwhacker sensor is active.
     *
     * @return true if weedwhacker sensor is active, false otherwise.
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
        weedWhackerTrigger.enableTrigger(this::triggerCallback);
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
        weedWhackerTrigger.disableTrigger();
    }   //disableTrigger

    /**
     * This method is called when the weedwhacker sensor is triggered.
     *
     * @param context specifies true if an object is captured, false otherwise.
     */
    private void triggerCallback(Object context)
    {
        final String funcName = "triggerCallback";
        sensorActive = ((AtomicBoolean) context).get();

        if (triggerEvent != null)
        {
            triggerEvent.signal();
        }

         if (robot.ledIndicator != null)
         {
             robot.ledIndicator.setHasObject(sensorActive);
         }

        if (msgTracer != null)
        {
            msgTracer.traceInfo(funcName, "[%.3f] active=%s", TrcTimer.getModeElapsedTime(), sensorActive);
        }
    }   //triggerCallback

}   //class WeedWhacker
