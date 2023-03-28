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
import TrcFrcLib.frclib.FrcCANFalcon;
import TrcFrcLib.frclib.FrcCANTalon;
import TrcFrcLib.frclib.FrcDigitalInput;
import TrcFrcLib.frclib.FrcPneumatic;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import team492.Robot;
import team492.RobotParams;

public class Intake implements TrcExclusiveSubsystem {
    private static final String moduleName = "Intake";

    private final Robot robot;
    private final TrcDbgTrace msgTracer;

    private final FrcCANFalcon intakeMotor;
    private final FrcDigitalInput intakeSensor;
    private final TrcTriggerDigitalInput intakeTrigger;
    private boolean sensorActive = false;
    private TrcEvent triggerEvent = null;

    public Intake(Robot robot, TrcDbgTrace msgTracer) {
        this.robot = robot;
        this.msgTracer = msgTracer;

        intakeMotor = new FrcCANFalcon("intakeMotor", RobotParams.CANID_INTAKE);

        intakeSensor = new FrcDigitalInput(moduleName + ".sensor", RobotParams.DIO_INTAKE_SENSOR);
        intakeSensor.setInverted(true);
        intakeTrigger = new TrcTriggerDigitalInput("intakeTrigger", intakeSensor);
    }

    /**
     * This method returns the state of the Arm in a string.
     */
    @Override
    public String toString()
    {
        return String.format(
            Locale.US, "%s: pwr=%.1f, hasObject=%s", moduleName, getMotorPower(), hasObject());
    }   //toString

    public double getMotorPower() {
        return intakeMotor.getMotorPower();
    }

    public void cancel(String owner)
    {
        if (validateOwnership(owner))
        {
            intakeMotor.stopMotor();
        }
    }   //cancel

    public void cancel()
    {
        cancel(null);
    }
    
    public void setPower(String owner, double delay, double power, double duration, TrcEvent event) {
        final String funcName = "setPower";

        if (msgTracer != null) {
            msgTracer.traceInfo(
                    funcName, "[%.3f] owner=%s, delay=%.1f, Power=%.1f, duration=%.3f",
                    TrcTimer.getModeElapsedTime(), owner, delay, power, duration);
        }

        if (validateOwnership(owner)) {
            intakeMotor.set(delay, power, duration, event);
        }
    }

    public void setPower(String owner, double delay, double power, double duration) {
        setPower(null, delay, power, duration, null);
    } // setPower

    public void setPower(double delay, double power, double duration) {
        setPower(null, delay, power, duration, null);
    } // setPower

    public void setPower(double power) {
        setPower(null, 0.0, power, 0.0, null);
    }

    public boolean hasObject() {
        return sensorActive;
    } // hasObject

    public void enableTrigger(TrcEvent event) {
        triggerEvent = event;
        if (triggerEvent != null) {
            triggerEvent.clear();
        }
        intakeTrigger.enableTrigger(this::intakeTriggerCallback);
    } // enableTrigger

    /**
     * This method disables the sensor trigger.
     */
    public void disableTrigger() {
        if (triggerEvent != null) {
            triggerEvent.cancel();
            triggerEvent = null;
        }
        intakeTrigger.disableTrigger();
    } // disableTrigger

    /**
     * This method is called when the intake sensor is triggered.
     *
     * @param context specifies true if an object is captured, false otherwise.
     */
    private void intakeTriggerCallback(Object context) {
        final String funcName = "intakeTriggerCallback";
        sensorActive = ((AtomicBoolean) context).get();

        if (sensorActive) {
            intakeMotor.stopMotor();
        }

        if (triggerEvent != null) {
            triggerEvent.signal();
        }

        if (robot.ledIndicator != null) {
            robot.ledIndicator.setHasObject(sensorActive);
        }

        if (msgTracer != null) {
            msgTracer.traceInfo(funcName, "[%.3f] active=%s", TrcTimer.getModeElapsedTime(), sensorActive);
        }
    } // intakeTriggerCallback
}