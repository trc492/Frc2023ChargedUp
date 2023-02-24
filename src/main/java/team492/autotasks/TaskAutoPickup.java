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
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcRobot.RunMode;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcTaskMgr.TaskType;
import TrcFrcLib.frclib.FrcPhotonVision.DetectedObject;
import team492.Robot;
import team492.RobotParams;
import team492.FrcAuto.ObjectType;
import team492.vision.PhotonVision.PipelineType;


public class TaskAutoPickup extends TrcAutoTask<TaskAutoPickup.State>
{
    private static final String moduleName = "TaskAutoPickup";

    public enum State
    {
        START,
        LOOK_FOR_TARGET,
        DRIVE_TO_TARGET,
        INTAKE_OBJECT,
        PICKUP_OBJECT,
        DONE
    }   //enum State

    private static class TaskParams
    {
        ObjectType objectType;
        boolean useVision;

        TaskParams(ObjectType objectType, boolean useVision)
        {
            this.objectType = objectType;
            this.useVision = useVision;
        }   //TaskParams
    }   //class TaskParams

    private final String owner;
    private final Robot robot;
    private final TrcDbgTrace msgTracer;
    private final TrcTimer timer;
    private final TrcEvent event;
    private String currOwner = null;
    private DetectedObject detectedTarget;
    private TrcPose2D robotPose;

    public TaskAutoPickup(String owner, Robot robot, TrcDbgTrace msgTracer)
    {
        super(moduleName, owner, TrcTaskMgr.TaskType.POST_PERIODIC_TASK, msgTracer);
        this.owner = owner;
        this.robot = robot;
        this.msgTracer = msgTracer;
        timer = new TrcTimer(moduleName + ".timer");
        event = new TrcEvent(moduleName + ".event");
    }   //TaskAutoPickup

    public void autoAssistPickup(ObjectType objectType, boolean useVision, TrcEvent completionEvent)
    {
        startAutoTask(State.START, new TaskParams(objectType, useVision), completionEvent);
    }
    
    public void autoAssistCancel()
    {
        stopAutoTask(false);
    }   //autoAssistCancel

    protected boolean acquireSubsystemsOwnership() 
    {
        boolean success = owner == null ||
                          (robot.robotDrive.driveBase.acquireExclusiveAccess(owner) &&
                           robot.elevatorPidActuator.acquireExclusiveAccess(owner) &&
                           robot.armPidActuator.acquireExclusiveAccess(owner) &&
                           robot.intake.acquireExclusiveAccess(currOwner));
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
            robot.elevatorPidActuator.releaseExclusiveAccess(currOwner);
            robot.armPidActuator.releaseExclusiveAccess(currOwner);
            robot.intake.releaseExclusiveAccess(currOwner);
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
        TaskParams taskParams = (TaskParams) params;
        switch(state)
        {
            case START:
                robot.grabber.release();
                robot.elevatorPidActuator.setTarget(currOwner, 0.0, true, 1.0, null, 0.0);
                robot.armPidActuator.setTarget(currOwner, 0.0, true, 1.0, null, 0.0);
                sm.setState(taskParams.useVision? State.LOOK_FOR_TARGET: State.INTAKE_OBJECT);
                break;

            case LOOK_FOR_TARGET:
                if (taskParams.objectType == ObjectType.CONE) {
                    robot.photonVision.setPipeline(PipelineType.CONE);
                } else {
                    robot.photonVision.setPipeline(PipelineType.CUBE);
                }
                boolean success = robot.photonVision.detectBestObject(event, 0.1);
                sm.waitForSingleEvent(event, (success? State.DRIVE_TO_TARGET: State.DONE)); //if fails to detect, goes to DONE
                break;
            
            case DRIVE_TO_TARGET:
                robotPose = robot.photonVision.getRobotFieldPosition(detectedTarget);
                robot.robotDrive.setFieldPosition(robotPose, false); //is this needed?
                detectedTarget = robot.photonVision.getLastDetectedBestObject();
                TrcPose2D target;
                if (taskParams.objectType == ObjectType.CONE) {
                    target = robot.photonVision.getTargetPose2D(detectedTarget, 12.81); //12 + 13/16
                } else {
                    target = robot.photonVision.getTargetPose2D(detectedTarget, 9.5);  //9.5 +- 0.5
                }
                robot.robotDrive.purePursuitDrive.start(
                    currOwner, event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                    target);
                sm.waitForSingleEvent(event, State.INTAKE_OBJECT);
                break;
            
            //don't believe APPROACH_OBJECT state was needed, deleted
            case INTAKE_OBJECT:
                double pickupTime = 2.0;
                robot.intake.extend();
                sm.waitForSingleEvent(event, State.PICKUP_OBJECT);
                timer.set(pickupTime, event);
                break;
            
            case PICKUP_OBJECT:
                robot.intake.retract();

                if (taskParams.objectType == ObjectType.CONE) {
                    robot.grabber.grabCone();
                } else {
                    robot.grabber.grabCube();
                }
                sm.setState(State.DONE);
                break;
            
            case DONE:
                stopAutoTask(true);
                break;
        }
    }   //runTaskState
}
