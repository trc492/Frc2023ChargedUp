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
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcTaskMgr.TaskType;
import TrcFrcLib.frclib.FrcPhotonVision.DetectedObject;
import team492.Robot;
import team492.RobotParams;
import team492.FrcAuto.ObjectType;
import team492.vision.PhotonVision.PipelineType;

/**
 * This class implements auto-assist task to pick up a cube or a cone from the ground.
 * Precondition: None
 */
public class TaskAutoPickup extends TrcAutoTask<TaskAutoPickup.State>
{
    private static final String moduleName = "TaskAutoPickup";

    public enum State
    {
        START,
        DRIVE_TO_OBJECT,
        PREP_FOR_PICKUP_ONLY,
        PICKUP_OBJECT,
        PICKUP_OBJECT_PICKUP_ONLY,
        PREP_FOR_TRAVEL,
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
    private final TrcEvent elevatorEvent;
    private final TrcEvent armEvent;
    private final TrcEvent intakeEvent;
    private final TrcEvent visionEvent;
    private final TrcEvent driveEvent;
    private final TrcEvent wristEvent;
    private String currOwner = null;
    private String currDriveOwner = null;

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
        elevatorEvent = new TrcEvent(moduleName + ".elevatorEvent");
        armEvent = new TrcEvent(moduleName + ".armEvent");
        intakeEvent = new TrcEvent(moduleName + ".intakeEvent");
        visionEvent = new TrcEvent(moduleName + ".visionEvent");
        driveEvent = new TrcEvent(moduleName + ".driveEvent");
        wristEvent = new TrcEvent(moduleName + ".wristEvent");
    }   //TaskAutoPickup

    /**
     * This method starts the auto-assist operation to pickup an object.
     *
     * @param objectType specifies the object type to pickup (cone or cube).
     * @param useVision specifies true to use vision assist, false otherwise.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoAssistPickup(
        ObjectType objectType, boolean useVision, TrcEvent completionEvent)
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
            robot.elevatorPidActuator.acquireExclusiveAccess(ownerName) &&
            robot.armPidActuator.acquireExclusiveAccess(ownerName) &&
            robot.intake.acquireExclusiveAccess(ownerName);
        // Don't acquire drive base ownership globally. Acquire it only if we need to drive.
        // For example, we don't need to drive in PickupOnly.
        // In PickupOnly, the driver can still drive while in the middle of autoPickup.
        if (success)
        {
            currOwner = ownerName;
            if (msgTracer != null)
            {
                msgTracer.traceInfo(funcName, "%s: Successfully acquired subsystem ownerships.", moduleName);
            }
        }
        else
        {
            if (msgTracer != null)
            {
                msgTracer.traceInfo(funcName, "%s: Failed to acquire subsystem ownership.", moduleName);
                TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
                msgTracer.traceInfo(
                    funcName,
                    "%s: Failed to acquire subsystem ownership (currOwner=%s, robotDrive=%s, elevator=%s, arm=%s, intake=%s).",
                    moduleName, currOwner, ownershipMgr.getOwner(robot.robotDrive.driveBase),
                    ownershipMgr.getOwner(robot.elevatorPidActuator), ownershipMgr.getOwner(robot.armPidActuator),
                    ownershipMgr.getOwner(robot.intake));
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

        if(currOwner != null)
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

            if (currDriveOwner != null)
            {
                robot.robotDrive.driveBase.releaseExclusiveAccess(currOwner);
                currDriveOwner = null;
            }

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
        final String funcName = "stopSubsystems";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(funcName, "%s: Stopping subsystems.", moduleName);
        }

        if (currDriveOwner != null)
        {
            robot.robotDrive.cancel(currDriveOwner);
        }

        robot.elevatorPidActuator.cancel(currOwner);
        robot.armPidActuator.cancel(currOwner);
        robot.intake.autoAssistCancel(currOwner);
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
        //  Preconditions:
        //      All subsystems are in "turtle mode" for traveling: arm at TRAVEL_POS, elevator at SAFE_HEIGHT,
        //      intake retracted.
        switch (state)
        {
            case START:
                // PickupOnly doesn't come to this state, meaning we will be driving towards the target thus requiring us to
                // acquire ownership of the drive base.
                if (acquireDriveBaseOwnership())
                {
                    // Assume pickup position depending on the object
                    // If useVision, call vision to detect object with timeout, signal event.
                    // Wait for all events, then goto DRIVE_TO_OBJECT.

                    robot.elevatorPidActuator.setPosition(RobotParams.ELEVATOR_MIN_POS, true, 1.0, elevatorEvent);
                    robot.armPidActuator.setPosition(RobotParams.ARM_LOW_POS, true, RobotParams.ARM_MAX_POWER, armEvent);
                    robot.wristPidActuator.setPosition(
                        taskParams.objectType == ObjectType.CUBE? RobotParams.WRIST_CUBE_PICKUP_POSITION:
                        RobotParams.WRIST_CONE_PICKUP_POSITION, true);

                    if (taskParams.useVision)
                    {
                        robot.photonVision.setPipeline(
                            taskParams.objectType == ObjectType.CUBE? PipelineType.CUBE: PipelineType.CONE);
                        robot.photonVision.detectBestObject(visionEvent, RobotParams.VISION_TIMEOUT);
                        sm.addEvent(visionEvent);
                    }

                    sm.addEvent(elevatorEvent);
                    sm.addEvent(armEvent);
                    sm.waitForEvents(State.DRIVE_TO_OBJECT, true);
                }
                else
                {
                    // Failed to acquire drive base ownership, quit.
                    sm.setState(State.DONE);
                }
                break;

            case DRIVE_TO_OBJECT:
                // Turn on intake
                // Arm intake sensor, signal event when has object.
                // If useVision and vision detected object use detected object position,
                //      otherwise set position to 60-inch forward.
                // Call purePursuit to go to object position, signal event.
                // Wait for either events, then goto PICKUP_OBJECT.

                robot.intake.autoAssistIntake(currOwner, 0.0, RobotParams.INTAKE_PICKUP_POWER, intakeEvent, 0.0);
                sm.addEvent(intakeEvent);

                robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.2);
                robot.robotDrive.purePursuitDrive.setRotOutputLimit(0.2);

                DetectedObject target;
                if (taskParams.useVision && (target = robot.photonVision.getLastDetectedBestObject()) != null)
                {
                    double targetDist = TrcUtil.magnitude(target.targetPoseFrom2D.x, target.targetPoseFrom2D.y);
                    double targetAngle = target.targetPoseFrom2D.angle;
                    robot.globalTracer.traceInfo(
                        moduleName, "Detected %s: targetDist=%s, targetAngle=%s", taskParams.objectType, targetDist, targetAngle);
                    robot.robotDrive.purePursuitDrive.start(
                        currOwner, driveEvent, 2.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(0.0, 6.0, targetAngle),
                        new TrcPose2D(0.0, targetDist + 48.0, 0.0));
                }
                else
                {
                    // No vision or no target, just drive forwards 5 feet
                    robot.robotDrive.purePursuitDrive.start(
                        currOwner, driveEvent, 2.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(0.0, 60.0, 0.0));
                }
                sm.addEvent(driveEvent);
                // We either have the object, or we missed.
                sm.waitForEvents(State.PREP_FOR_TRAVEL);
                break;
            
            case PREP_FOR_TRAVEL:
                // Assume travel position
                robot.elevatorPidActuator.setPosition(
                    currOwner, 0.0, RobotParams.ELEVATOR_TRAVEL_POSITION, true, 1.0, null, 0.0);
                robot.armPidActuator.setPosition(
                    currOwner, 0.0, RobotParams.ARM_MIN_POS, true, RobotParams.ARM_MAX_POWER, armEvent, 0.0);
                robot.wristPidActuator.setPosition(
                    currOwner, 0.0, RobotParams.WRIST_TRAVEL_POSITION, true, 1.0, wristEvent, 0.0);
                sm.waitForSingleEvent(armEvent, State.DONE);
                break;

            default:
            case DONE:
                robot.robotDrive.purePursuitDrive.cancel();
                robot.intake.autoAssistCancel(currOwner);
                // Stop task.
                stopAutoTask(true);
                break;
        }
    }   //runTaskState

    /**
     * This method acquires drive base ownership if necessary.
     *
     * @return true if successful, false otherwise.
     */
    private boolean acquireDriveBaseOwnership()
    {
        final String funcName = "acquireDriveBaseOwnership";
        boolean success = true;

        if (ownerName != null)
        {
            // It's not pickup only, meaning we will be driving towards the target thus requiring us to
            // acquire ownership of the drive base.
            if (robot.robotDrive.driveBase.acquireExclusiveAccess(ownerName))
            {
                currDriveOwner = ownerName;
            }
            else
            {
                if (msgTracer != null)
                {
                    msgTracer.traceInfo(funcName, "Failed to acquire drive base ownership.");
                }
                success = false;
            }
        }

        return success;
    }   //acquireDriveBaseOwnership

}   //class TaskAutoPickup
