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
import TrcCommonLib.trclib.TrcOwnershipMgr;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot.RunMode;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcTaskMgr.TaskType;
import TrcFrcLib.frclib.FrcPhotonVision.DetectedObject;
import team492.Robot;
import team492.RobotParams;
import team492.FrcAuto.ObjectType;
import team492.vision.PhotonVision.PipelineType;

/**
 * This class implements auto-assist task to pick up a cube or a cone from the ground.
 */
public class TaskAutoPickup extends TrcAutoTask<TaskAutoPickup.State>
{
    private static final String moduleName = "TaskAutoPickup";

    public enum State
    {
        START,
        MOVE_ARM_FORWARD,
        LOOK_FOR_TARGET,
        DRIVE_TO_TARGET,
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

    private final String ownerName;
    private final Robot robot;
    private final TrcDbgTrace msgTracer;
    private final TrcEvent event, intakeEvent;
    private String currOwner = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param ownerName specifies the owner name to take subsystem ownership, can be null if no ownership required.
     * @param robot specifies the robot object that contains all the necessary subsystems.
     * @param msgTracer specifies the tracer to use to log events, can be null if not provided.
     */
    public TaskAutoPickup(String ownerName, Robot robot, TrcDbgTrace msgTracer)
    {
        super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK, msgTracer);
        this.ownerName = ownerName;
        this.robot = robot;
        this.msgTracer = msgTracer;
        event = new TrcEvent(moduleName);
        intakeEvent = new TrcEvent(moduleName + ".intakeEvent");
    }   //TaskAutoPickup

    /**
     * This method starts the auto-assist operation to pickup an object.
     *
     * @param objectType specifies the object type to pickup (cone or cube).
     * @param useVision specifies true to use vision assist, false otherwise.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoAssistPickup(ObjectType objectType, boolean useVision, TrcEvent completionEvent)
    {
        final String funcName = "autoAssistPickup";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(
                funcName, "%s: objectType=%s, useVision=%s, event=%s",
                moduleName, objectType, useVision, completionEvent);
        }

        startAutoTask(State.START, new TaskParams(objectType, useVision), completionEvent);
    }   //autoAssistPickup

    /**
     * This method cancels an in progress auto-assist operation if any.
     */
    public void autoAssistCancel()
    {
        final String funcName = "autoAssistCancel";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(funcName, "%s: Canceling auto-assist pickup.", moduleName);
        }
        stopAutoTask(false);
    }   //autoAssistCancel

    //
    // Implement TrcAutoTask abstract methods.
    //

    /**
     * This method is called by the super class to acquire ownership of all subsystems involved in the auto-assist
     * operation. This is typically done before starting an auto-assist operation.
     *
     * @return true if acquired all subsystems ownership, false otherwise. It releases all ownership if any acquire
     *         failed.
     */
    @Override
    protected boolean acquireSubsystemsOwnership() 
    {
        final String funcName = "acquireSubsystemsOwnership";
        boolean success = ownerName == null ||
                          (robot.robotDrive.driveBase.acquireExclusiveAccess(ownerName) &&
                           robot.elevatorPidActuator.acquireExclusiveAccess(ownerName) &&
                           robot.armPidActuator.acquireExclusiveAccess(ownerName) &&
                           robot.intake.acquireExclusiveAccess(ownerName));
        if (success)
        {
            currOwner = ownerName;
        }
        else
        {
            if (msgTracer != null)
            {
                msgTracer.traceInfo(funcName, "%s: Failed to acquire subsystem ownership.", moduleName);
            }
            releaseSubsystemsOwnership();
        }

        return success;
    }   //acquireSubsystemsOwnership

    /**
     * This method is called by the super class to release ownership of all subsystems involved in the auto-assist
     * operation. This is typically done if the auto-assist operation is completed or canceled.
     */
    @Override
    protected void releaseSubsystemsOwnership()
    {
        final String funcName = "releaseSubsystemsOwnership";

        if(ownerName != null)
        {
            if (msgTracer != null)
            {
                TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
                msgTracer.traceInfo(
                    funcName,
                    "%s: Releasing subsystem ownership (currOwner=%s, robotDrive=%s, elevator=%s, arm=%s, intake=%s).",
                    moduleName, currOwner, ownershipMgr.getOwner(robot.robotDrive.driveBase),
                    ownershipMgr.getOwner(robot.elevatorPidActuator), ownershipMgr.getOwner(robot.armPidActuator),
                    ownershipMgr.getOwner(robot.intake));
            }
            robot.robotDrive.driveBase.releaseExclusiveAccess(currOwner);
            robot.elevatorPidActuator.releaseExclusiveAccess(currOwner);
            robot.armPidActuator.releaseExclusiveAccess(currOwner);
            robot.intake.releaseExclusiveAccess(currOwner);
            currOwner = null;
        }
    }   //releaseSubsystemsOwnership

    /**
     * This method is called by the super class to stop all the subsystems.
     */
    @Override
    protected void stopSubsystems()
    {
        robot.robotDrive.cancel(currOwner);
        robot.elevatorPidActuator.cancel(currOwner);
        robot.armPidActuator.cancel(currOwner);
        robot.intake.cancel(currOwner);
    }   //stopSubsystems

    /**
     * This methods is called periodically to run the auto-assist task.
     *
     * @param params specifies the task parameters.
     * @param state specifies the current state of the task.
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    @Override
    protected void runTaskState(
        Object params, State state, TaskType taskType, RunMode runMode, boolean slowPeriodicLoop)
    {
        TaskParams taskParams = (TaskParams) params;

        switch (state)
        {
            case START:
                robot.grabber.releaseAll();
                // TODO (Code Review): What is preset[3]? Why? I thought elevator should be retracted?! Are you thinking
                // the arm is still tucked inside the belly of the robot? As a pre-condition, you may say the arm should
                // be out already.
                robot.elevatorPidActuator.setPosition(
                    currOwner, 0, RobotParams.elevatorPresets[3], true, 1.0, event, 0);
                sm.waitForSingleEvent(event, State.MOVE_ARM_FORWARD);
                break;

            case MOVE_ARM_FORWARD:
                robot.armPidActuator.setPosition(
                    currOwner, RobotParams.ARM_TRAVEL_POSITION, true, 1.0, event, 0);
                sm.waitForSingleEvent(event, taskParams.useVision? State.LOOK_FOR_TARGET: State.DRIVE_TO_TARGET);
                break;

            case LOOK_FOR_TARGET:
                robot.photonVision.setPipeline(
                    taskParams.objectType == ObjectType.CUBE? PipelineType.CUBE: PipelineType.CONE);
                robot.photonVision.detectBestObject(event, RobotParams.VISION_TIMEOUT);
                sm.waitForSingleEvent(event, State.DRIVE_TO_TARGET);
                break;
            
            case DRIVE_TO_TARGET:
                // TODO (Code Review): Do we really need this?
                // robot.elevatorPidActuator.setPosition(
                //     currOwner, 0, 0, true, 1.0, event, 0);
                TrcPose2D target = null;
                if (taskParams.useVision)
                {
                    // Check if vision has detected a target.
                    DetectedObject detectedTarget = robot.photonVision.getLastDetectedBestObject();
                    if (detectedTarget != null)
                    {
                        // Cone center height: 12+13/16, Cube center height: 9.5 +/- 0.5.
                        target = robot.photonVision.getTargetPose2D(
                            detectedTarget, detectedTarget.getRect().height / 2.0);
                        target.angle = 0.0;     // Just need x and y but maintain the current heading.
                        //target.y += 12; not sure how much to add at the moment, need testing
                    }
                }

                if (target == null)
                {
                    target = new TrcPose2D(0.0, 36.0);
                }

                robot.intake.extend();
                robot.intake.setPower(currOwner, 0.0, 1.0, 1.0, 0.0);
                robot.intake.setTriggerEnabled(true, intakeEvent);
                sm.addEvent(intakeEvent);

                robot.robotDrive.purePursuitDrive.start(
                    currOwner, event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), true, target);
                sm.addEvent(event);

                sm.waitForEvents(State.PICKUP_OBJECT);
                break;

            case PICKUP_OBJECT:
                // TODO (Code Review): Picking up an object requires a sequence that Eric has published a document for.
                // Please consult Eric's document. It's not as simple as this.
                // Tried to abide by Eric's document as much as possible, but parts of it (rotating cone to be nose out) was too challenging/unthinkable!!
                robot.armPidActuator.setPosition(currOwner, RobotParams.ARM_PICKUP_POSITION, true, 1.0, null, 0);
                robot.intake.cancel(currOwner);
                robot.robotDrive.purePursuitDrive.cancel(currOwner);
                robot.intake.setTriggerEnabled(false, null);
                // TODO (CodeReview): Do we need to retract here? We haven't grab the object yet.
                robot.intake.retract();

                if (taskParams.objectType == ObjectType.CONE)
                {
                    robot.grabber.grabCone();
                }
                else
                {
                    robot.grabber.grabCube();
                }
                // TODO (Code Review): Need to wait until you firmly grabbed the object before moving the elevator. And why
                // is the elevator height 0?
                robot.elevatorPidActuator.setPosition(currOwner, 0, 0, true, 1.0, event, 1);
                sm.waitForSingleEvent(event, State.DONE);
                break;
            
            case DONE:
                stopAutoTask(true);
                break;
        }
    }   //runTaskState

}   //class TaskAutoPickup
