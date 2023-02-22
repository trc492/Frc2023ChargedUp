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

package team492.autotasks;

import TrcCommonLib.trclib.TrcAutoTask;
import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcRobot.RunMode;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcTaskMgr.TaskType;
import team492.Robot;
import team492.RobotParams;


public class TaskAutoPickup extends TrcAutoTask<TaskAutoPickup.State>
{
    private static final String moduleName = "TaskAutoPickup";

    public enum State
    {
        START,
        LOOK_FOR_TARGET,
        DRIVE_TO_TARGET,
        APPROACH_OBJECT,
        INTAKE_OBJECT,
        PICKUP_OBJECT,
        DONE
    }   //enum State

    public enum ObjectType
    {
        CUBE, 
        CONE
    }   //enum ObjectType
    
    private final String owner;
    private final Robot robot;
    private final TrcDbgTrace msgTracer;
    private final TrcTimer timer;
    private final TrcEvent event;
    private String currOwner = null;

    public TaskAutoPickup(String owner, Robot robot, TrcDbgTrace msgTracer)
    {
        super(moduleName, owner, TrcTaskMgr.TaskType.POST_PERIODIC_TASK, msgTracer);
        this.owner = owner;
        this.robot = robot;
        this.msgTracer = msgTracer;
        timer = new TrcTimer(moduleName + ".timer");
        event = new TrcEvent(moduleName + ".event");
    }   //TaskAutoPickup

    public void autoAssistCancel()
    {
        stopAutoTask(false);
    }   //autoAssistCancel

    protected boolean acquireSubsystemsOwnership() 
    {
        boolean success = owner == null || robot.robotDrive.driveBase.acquireExclusiveAccess(owner);
        if (success)
        {
            currOwner = owner;
        }
        else
        {
            releaseSubsystemsOwnership();
        }
        return success;
    }   //acquireSubsystemsOwnership

    protected void releaseSubsystemsOwnership()
    {
        if(owner != null)
        {
            robot.robotDrive.driveBase.releaseExclusiveAccess(currOwner);
            currOwner = null;
        }
    }   //releaseSubsystemsOwnership

    protected void stopSubsystems()
    {
        robot.robotDrive.cancel(currOwner);
        robot.elevatorPidActuator.cancel(currOwner);
        robot.armPidActuator.cancel(currOwner);
        robot.intake.cancel(currOwner);
    }   //stopSubsystems

    protected void runTaskState(
        Object params, State state, TaskType taskType, RunMode runMode, boolean slowPeriodicLoop)
    {
        switch(state)
        {
            case START:
                sm.setState(RobotParams.Preferences.useLimeLightVision? State.LOOK_FOR_TARGET: State.INTAKE_OBJECT);
                break;

            case LOOK_FOR_TARGET:
                
                sm.setState(State.DRIVE_TO_TARGET);
                break;
            
            case DRIVE_TO_TARGET:

                sm.setState(State.APPROACH_OBJECT);
                break;
            
            case APPROACH_OBJECT:

                sm.setState(State.INTAKE_OBJECT);
                break;
            
            case INTAKE_OBJECT:
                double pickupTime = 1.0;
                robot.intake.extend();
                sm.waitForSingleEvent(event, State.PICKUP_OBJECT);
                timer.set(pickupTime, event);
                break;
            
            case PICKUP_OBJECT:

                sm.setState(State.DONE);
                break;
            
            case DONE:
                stopAutoTask(true);
                break;
        }
    }   //runTaskState
}
