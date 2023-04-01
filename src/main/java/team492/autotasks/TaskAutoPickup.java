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
    private final TrcEvent wristEvent;
    private final TrcEvent intakeEvent;
    private final TrcEvent visionEvent;
    private final TrcEvent driveEvent;
    private String currOwner = null;
    private String driveOwner = null;

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
        wristEvent = new TrcEvent(moduleName + ".wristEvent");
        intakeEvent = new TrcEvent(moduleName + ".intakeEvent");
        visionEvent = new TrcEvent(moduleName + ".visionEvent");
        driveEvent = new TrcEvent(moduleName + ".driveEvent");
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
                robot.wristPidActuator.acquireExclusiveAccess(ownerName) &&
                robot.intake.acquireExclusiveAccess(ownerName);
        // Don't acquire drive base ownership globally. Acquire it only if we need to drive.
        // For example, we only need to drive if we are using vision to approach the object.
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
                TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
                msgTracer.traceInfo(
                    funcName,
                    "%s: Failed to acquire subsystem ownership (currOwner=%s, robotDrive=%s, elevator=%s, arm=%s, wrist=%s, intake=%s).",
                    moduleName, currOwner, ownershipMgr.getOwner(robot.robotDrive.driveBase),
                    ownershipMgr.getOwner(robot.elevatorPidActuator), ownershipMgr.getOwner(robot.armPidActuator),
                    ownershipMgr.getOwner(robot.wristPidActuator), ownershipMgr.getOwner(robot.intake));
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
                    "%s: Releasing subsystem ownership (currOwner=%s, robotDrive=%s, elevator=%s, arm=%s, wrist=%s, intake=%s).",
                    moduleName, currOwner, ownershipMgr.getOwner(robot.robotDrive.driveBase),
                    ownershipMgr.getOwner(robot.elevatorPidActuator), ownershipMgr.getOwner(robot.armPidActuator),
                    ownershipMgr.getOwner(robot.wristPidActuator), ownershipMgr.getOwner(robot.intake));
            }

            if (driveOwner != null)
            {
                robot.robotDrive.driveBase.releaseExclusiveAccess(driveOwner);
                driveOwner = null;
            }

            robot.elevatorPidActuator.releaseExclusiveAccess(currOwner);
            robot.armPidActuator.releaseExclusiveAccess(currOwner);
            robot.wristPidActuator.releaseExclusiveAccess(currOwner);
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

        if (driveOwner != null)
        {
            robot.robotDrive.cancel(driveOwner);
        }

        robot.elevatorPidActuator.cancel(currOwner);
        robot.armPidActuator.cancel(currOwner);
        robot.wristPidActuator.cancel(currOwner);
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
        State nextState;
        //  Preconditions:
        //      All subsystems are in "turtle mode" for traveling: arm at TRAVEL_POS, elevator at SAFE_HEIGHT,
        //      intake retracted.
        switch (state)
        {
            case START:
                double elevatorPos, armPos, wristPos;
                boolean useWeedWhacker = false;
                // Determine arm, elevator wrist positions based on pick up objectType.
                if (taskParams.objectType == ObjectType.CONE)
                {
                    elevatorPos = 0.0;
                    armPos = 0.0;
                    wristPos = 0.0;
                }
                else
                {
                    elevatorPos = 0.0;
                    armPos = 0.0;
                    wristPos = 0.0;
                    useWeedWhacker = robot.weedWhacker != null;
                }

                robot.elevatorPidActuator.setPosition(
                    currOwner, 0.0, elevatorPos, true, 1.0, elevatorEvent, 0.7);
                sm.addEvent(elevatorEvent);
                robot.armPidActuator.setPosition(
                    currOwner, 0.0, armPos, true, RobotParams.ARM_MAX_POWER, armEvent, 0.7);
                sm.addEvent(armEvent);
                robot.wristPidActuator.setPosition(
                    currOwner, 0.0, wristPos, true, RobotParams.WRIST_MAX_POWER, wristEvent, 0.7);
                sm.addEvent(wristEvent);

                if (useWeedWhacker)
                {
                    robot.weedWhacker.extend();
                    robot.weedWhacker.setPower(currOwner, 0.0, RobotParams.WEEDWHACKER_CUBE_PICKUP_POWER, 0.0, null);
                }

                if (taskParams.useVision)
                {
                    robot.photonVision.setPipeline(
                        taskParams.objectType == ObjectType.CONE? PipelineType.CONE: PipelineType.CUBE);
                    robot.photonVision.detectBestObject(visionEvent, RobotParams.VISION_TIMEOUT);
                    sm.addEvent(visionEvent);
                    nextState = State.DRIVE_TO_OBJECT;
                }
                else
                {
                    // If we are not using vision, the driver has control of driving. So, turn on intake.
                    robot.intake.autoAssistIntake(
                        currOwner, 0.0,
                        taskParams.objectType == ObjectType.CONE?
                            RobotParams.INTAKE_PICKUP_POWER: -RobotParams.INTAKE_PICKUP_POWER,
                        taskParams.objectType == ObjectType.CONE?
                            RobotParams.INTAKE_CONE_RETAIN_POWER: 0.0,
                        0.5, intakeEvent, 0.0);
                    sm.addEvent(intakeEvent);
                    nextState = State.PREP_FOR_TRAVEL;
                }

                sm.waitForEvents(nextState, true);
                break;

            case DRIVE_TO_OBJECT:
                if (acquireDriveBaseOwnership())
                {
                    DetectedObject detectedTarget = robot.photonVision.getLastDetectedBestObject();

                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.2);
                    robot.robotDrive.purePursuitDrive.setRotOutputLimit(0.2);
                    if (detectedTarget != null)
                    {
                        double targetDist = TrcUtil.magnitude(
                            detectedTarget.targetPoseFrom2D.x, detectedTarget.targetPoseFrom2D.y);
                        double targetAngle = detectedTarget.targetPoseFrom2D.angle;

                        robot.globalTracer.traceInfo(
                            moduleName, "Detected %s: targetDist=%s, targetAngle=%s",
                            taskParams.objectType, targetDist, targetAngle);
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
                    robot.intake.autoAssistIntake(
                        currOwner, 0.0,
                        taskParams.objectType == ObjectType.CONE?
                            RobotParams.INTAKE_PICKUP_POWER: -RobotParams.INTAKE_PICKUP_POWER,
                        taskParams.objectType == ObjectType.CONE?
                            RobotParams.INTAKE_CONE_RETAIN_POWER: 0.0,
                        0.5, intakeEvent, 0.0);
                    sm.addEvent(intakeEvent);
                    sm.waitForEvents(State.PREP_FOR_TRAVEL, false);
                }
                else
                {
                    // Failed to acquire drive base ownership for some reason. Let's quit.
                    sm.setState(State.DONE);
                }
                break;

            case PREP_FOR_TRAVEL:
                // Assume travel position.
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
                driveOwner = ownerName;
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
