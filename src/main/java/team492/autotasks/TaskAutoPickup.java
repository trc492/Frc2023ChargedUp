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
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcRobot.RunMode;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcTaskMgr.TaskType;
import TrcFrcLib.frclib.FrcPhotonVision.DetectedObject;
import team492.Robot;
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

    private final String ownerName;
    private final Robot robot;
    private final TrcDbgTrace msgTracer;
    private final TrcTimer timer;
    private final TrcEvent event, armEvent, elevatorEvent;
    private String currOwner = null;
    private DetectedObject detectedTarget;
    private TrcPose2D robotPose;

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
        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        armEvent = new TrcEvent(moduleName);
        elevatorEvent = new TrcEvent(moduleName);
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

        switch(state)
        {
            case START:
                // Setup subsystems.
                robot.grabber.releaseAll();
                robot.elevatorPidActuator.zeroCalibrate();
                robot.armPidActuator.zeroCalibrate();
                sm.setState(taskParams.useVision? State.LOOK_FOR_TARGET: State.DRIVE_TO_TARGET);
                break;

            case LOOK_FOR_TARGET:
                robot.photonVision.setPipeline(
                    taskParams.objectType == ObjectType.CUBE? PipelineType.CUBE: PipelineType.CONE);
                robot.photonVision.detectBestObject(event, 0.5);
                sm.waitForSingleEvent(event, State.DRIVE_TO_TARGET);
                break;
            
            case DRIVE_TO_TARGET:
                // Check if vision has detected a target.
                robot.intake.extend();
                robot.intake.setPower(currOwner, 0.0, 1.0, 1.0, 0.0);
                // robot.intake.setTriggerEnabled(true, event);
               TrcPose2D relative;
                if (taskParams.useVision)
                {
                    robotPose = robot.photonVision.getRobotFieldPosition(detectedTarget);
                    //robot.robotDrive.setFieldPosition(robotPose, false); is this needed?
                    detectedTarget = robot.photonVision.getLastDetectedBestObject();
                    if (detectedTarget != null)
                    {
                        // DONE, needs review
                        // TODO (Code Review): In your scenario, it is different from AutoScore because vision detecting
                        // cones and cubes are not as reliable as AprilTag. So you should just get the relative position
                        // of the target from the camera and use "incremental" purePursuit to approach the object.
                        // The following code needs to be rewritten. Let's talk about how to approach this.
                        TrcPose2D target;
                        if (taskParams.objectType == ObjectType.CONE) {
                            target = robot.photonVision.getTargetPose2D(detectedTarget, 12.81); //12 + 13/16
                        } else {
                            target = robot.photonVision.getTargetPose2D(detectedTarget, 9.5);  //9.5 +- 0.5
                        }
                        relative = target.relativeTo(robotPose);
                    }
                    else
                    {
                        // DONE, needs review
                        // TODO (Code Review): Giving up so easily??? If vision doesn't see it, you may want to just go
                        // forward for a distance hoping to grab it anyway.
                        relative = new TrcPose2D(0, 60);
                    }
                }
                else
                {
                    // DONE, needs review
                    // TODO (Code Review): Giving up so easily??? If vision doesn't see it, you may want to just go
                    // forward for a distance hoping to grab it anyway.
                    relative = new TrcPose2D(0, 60);
                }
                robot.robotDrive.purePursuitDrive.start(
                    currOwner, event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                    relative);
                sm.waitForSingleEvent(event, State.PICKUP_OBJECT);
                // sm.waitForEvents(State.PICKUP_OBJECT);
                break;

            // DONE, needs review
            // TODO (Code Review): Intake (aka Weedwhacker) will have a sensor to detect if intake has something.
            // Therefore, don't use a timer, use the sensor to generate the event instead. Also, turning on the
            // weedwhacker must be accompanied by moving forward. Therefore, this state should really be combined
            // with the previous state. The previous state should do the following:
            // - Turn on the weedwhacker.
            // - Check if vision has detected something.
            // - If it did, use the detected distance info, otherwise just go forward some distance.
            // - Do a waitForEvents for two events, purePursuit completed the "moving forward" or weedwhacker has
            //   detected something in possession. In either case, move to the next state to "pick up".
            //case INTAKE_OBJECT:
            //    double pickupTime = 2.0;
            //    robot.intake.extend();
            //    sm.waitForSingleEvent(event, State.PICKUP_OBJECT);
            //    timer.set(pickupTime, event);
            //    break;
            
            case PICKUP_OBJECT:
                robot.intake.cancel(currOwner);
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
