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
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcTriggerDigitalInput;
import TrcFrcLib.frclib.FrcDigitalInput;
import TrcFrcLib.frclib.FrcPneumatic;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import team492.Robot;
import team492.RobotParams;

public class Grabber
{
    private static final String moduleName = "Grabber";

    private final Robot robot;
    private final TrcDbgTrace msgTracer;
    private final FrcPneumatic coneGrabber;
    private final FrcPneumatic cubeGrabber;
    private final FrcPneumatic cubePoker;
    private final FrcDigitalInput grabberSensor;
    private final TrcTriggerDigitalInput grabberTrigger;
    private TrcEvent triggerEvent = null;
    private boolean sensorActive = false;

    /**
     * Constructor: Create an instance of the object.
     *
     */
    public Grabber(Robot robot, TrcDbgTrace msgTracer)
    {
        this.robot = robot;
        this.msgTracer = msgTracer;

        coneGrabber = new FrcPneumatic(
            moduleName + ".cone", RobotParams.CANID_PCM, PneumaticsModuleType.REVPH,
            RobotParams.PNEUMATIC_CONE_GRABBER_RETRACT, RobotParams.PNEUMATIC_CONE_GRABBER_EXTEND);
        cubeGrabber = new FrcPneumatic(
            moduleName + ".cube", RobotParams.CANID_PCM, PneumaticsModuleType.REVPH,
            RobotParams.PNEUMATIC_CUBE_GRABBER_RETRACT, RobotParams.PNEUMATIC_CUBE_GRABBER_EXTEND);
        cubePoker = new FrcPneumatic(
            moduleName + ".poker", RobotParams.CANID_PCM, PneumaticsModuleType.REVPH,
            RobotParams.PNEUMATIC_POKER_EXTEND, RobotParams.PNEUMATIC_POKER_RETRACT);

        grabberSensor = new FrcDigitalInput(moduleName + ".sensor", RobotParams.DIO_GRABBER_SENSOR);
        grabberSensor.setInverted(false);
        grabberTrigger = new TrcTriggerDigitalInput(moduleName + ".trigger", grabberSensor);
        enableTrigger(null);
    }   //Grabber

    /**
     * This method returns the state of the Arm in a string.
     */
    @Override
    public String toString()
    {
        return String.format(
            Locale.US, "%s: grabbedCube=%s, grabbedCone=%s, poked=%s, hasObject=%s",
            moduleName, grabbedCube(), grabbedCone(), poked(), hasObject());
    }   //toString

    //This method is called to grab a cube, extends the left pneumatic for a partial grab
    public void grabCube()
    {   
        cubeGrabber.extend();
    }   //grabCube

    public void grabCube(double delay)
    {
        cubeGrabber.extend(delay);
    }   //grabCube

    public void grabCube(TrcEvent event)
    {
        cubeGrabber.extend(RobotParams.GRABBER_GRAB_DURATION, event);
    }   //grabCube

    public void releaseCube()
    {
        cubeGrabber.retract();
    }   //releaseCube

    public void releaseCube(TrcEvent event)
    {
        cubeGrabber.retract(RobotParams.GRABBER_RELEASE_DURATION, event);
    }   //releaseCube

    public void extendPoker()
    {
        cubePoker.extend();
    }

    public void extendPoker(double delay)
    {
        cubePoker.extend(delay);
    }

    public void retractPoker()
    {
        cubePoker.retract();
    }

    public void retractPoker(double delay)
    {
        cubePoker.retract(delay);
    }

    //This method is called to grab a cone, extends both pneumatics for a complete grab
    public void grabCone()
    {
        coneGrabber.extend();
    }   //grabCone

    public void grabCone(double delay)
    {
        coneGrabber.extend(delay);
    }   //grabCone

    public void grabCone(TrcEvent event)
    {
        coneGrabber.extend(RobotParams.GRABBER_GRAB_DURATION, event);
    }   //grabCone

    public void releaseCone()
    {
        coneGrabber.retract();
    }   //releaseCone

    public void releaseCone(double delay)
    {
        coneGrabber.retract(delay);
    }   //releaseCone

    public void releaseCone(TrcEvent event)
    {
        coneGrabber.retract(RobotParams.GRABBER_RELEASE_DURATION, event);
    }   //releaseCone

    //This method is called to open the grabber, retracting the pneumatics
    public void releaseAll()
    {
        coneGrabber.retract();
        cubeGrabber.retract();
    }   //releaseAll

    public boolean grabbedCube()
    {
        return cubeGrabber.isExtended();
    }   //grabbedCube

    public boolean grabbedCone()
    {
        return coneGrabber.isExtended();
    }   //grabbedCone

    public boolean poked()
    {
        return cubePoker.isExtended();
    }

    /**
     * This method checks if the grabber sensor is active.
     *
     * @return true if grabber sensor is active, false otherwise.
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
        grabberTrigger.enableTrigger(this::grabberTriggerCallback);
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
        grabberTrigger.disableTrigger();
    }   //disableTrigger

    /**
     * This method is called when the grabber sensor is triggered.
     *
     * @param context specifies true if an object is captured, false otherwise.
     */
    private void grabberTriggerCallback(Object context)
    {
        final String funcName = "grabberTriggerCallback";
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
    }   //grabberTriggerCallback

}   //class Grabber
