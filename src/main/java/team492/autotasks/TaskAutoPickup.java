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
import TrcCommonLib.trclib.TrcTimer;
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
        PICKUP_OBJECT,
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
    private final TrcEvent elevatorEvent, armEvent, intakeEvent, visionEvent, event;
    private final TrcTimer timer;
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
        elevatorEvent = new TrcEvent(moduleName + ".elevatorEvent");
        armEvent = new TrcEvent(moduleName + ".armEvent");
        intakeEvent = new TrcEvent(moduleName + ".intakeEvent");
        visionEvent = new TrcEvent(moduleName + ".visionEvent");
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName + ".timer");
    }   //TaskAutoPickup

    /**
     * This method starts the auto-assist operation to pickup an object.
     *
     * @param objectType specifies the object type to pickup (cone or cube).
     * @param useVision specifies true to use vision assist, false otherwise.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void 
    autoAssistPickup(ObjectType objectType, boolean useVision, TrcEvent completionEvent)
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
                          robot.robotDrive.driveBase.acquireExclusiveAccess(ownerName) &&
                          robot.elevatorPidActuator.acquireExclusiveAccess(ownerName) &&
                          robot.armPidActuator.acquireExclusiveAccess(ownerName) &&
                          robot.intake.acquireExclusiveAccess(ownerName);
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
        //  Preconditions:
        //      All subsystems are in "turtle mode" for travelling: arm at TRAVEL_POS, elevator at 0.0,
        //      whacker retracted.
        switch (state)
        {
            case START:
                // Set Intake down.
                // If picking up cube, open both cube and cone grabber,
                //      otherwise close cube grabber and open cone grabber.
                // Raise elevator to 6-inch, signal event.
                // Raise arm to 15-deg, signal event.
                // If useVision, call vision to detect object with timeout, signal event.
                // Wait for all events, then goto DRIVE_TO_OBJECT.
                robot.intake.extend();
                if (taskParams.objectType == ObjectType.CUBE)
                {
                    robot.grabber.releaseCube();
                }
                else
                {
                    robot.grabber.grabCube();
                }
                robot.grabber.releaseCone();

                robot.elevatorPidActuator.setPosition(
                    currOwner, 0.0, 6.0, true, 1.0, elevatorEvent, 0.0);
                sm.addEvent(elevatorEvent);
                robot.armPidActuator.setPosition(
                    currOwner, 0.0, 15.0, true, RobotParams.ARM_MAX_POWER, armEvent, 0.0);
                sm.addEvent(armEvent);

                if (taskParams.useVision)
                {
                    robot.photonVision.setPipeline(
                        taskParams.objectType == ObjectType.CUBE? PipelineType.CUBE: PipelineType.CONE);
                    robot.photonVision.detectBestObject(visionEvent, RobotParams.VISION_TIMEOUT);
                    sm.addEvent(visionEvent);
                }

                sm.waitForEvents(State.DRIVE_TO_OBJECT, 0.0, true);
                break;
            
            case DRIVE_TO_OBJECT:
                // Turn on whacker with speed appropriate for cube or cone.
                // Arm intake sensor, signal event when has object.
                // If useVision and vision detected object use detected object position,
                //      otherwise set position to 36-inch forward.
                // Call purePursuit to go to object position, signal event.
                // Wait for either events, then goto PICKUP_OBJECT.
                TrcPose2D targetPos = null;
                double intakePower = taskParams.objectType == ObjectType.CUBE?
                    RobotParams.INTAKE_CUBE_PICKUP_POWER: RobotParams.INTAKE_CONE_PICKUP_POWER;

                robot.intake.setPower(currOwner, 0.0, intakePower, intakePower, 0.0);
                robot.intake.setTriggerEnabled(true, intakeEvent);
                sm.addEvent(intakeEvent);

                if (taskParams.useVision)
                {
                    DetectedObject target = robot.photonVision.getLastDetectedBestObject();
                    if (target != null)
                    {
                        targetPos = robot.photonVision.getTargetPose2D(target, target.getRect().height/2.0);
                    }
                }

                if (targetPos == null)
                {
                    targetPos = new TrcPose2D(0.0, 36.0, 0.0);
                }

                robot.robotDrive.purePursuitDrive.start(
                    currOwner,event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), true, targetPos);
                sm.addEvent(event);

                sm.waitForEvents(State.PICKUP_OBJECT);
                break;
            
            case PICKUP_OBJECT:
                // Cancel purePursuit drive.
                // Stop intake.
                // Unarm intake sensor.
                // If intake does not have object, goto DONE, otherwise continue the following.
                // Lower elevator to 3-inch.
                // Lower arm to 5-deg.
                // If picking up cone, close cone grabber with a delay, otherwise close cube grabber with a delay.
                // Set a delay timer and signal event.
                // Wait for timer event then goto PREP_FOR_TRAVEL.
                robot.robotDrive.purePursuitDrive.cancel();
                robot.intake.cancel();
                robot.intake.setTriggerEnabled(false, null);

                if (robot.intake.hasObject())
                {
                    robot.elevatorPidActuator.setPosition(
                        currOwner, 0.0, 3.0, true, 1.0, null, 0.0);
                    robot.armPidActuator.setPosition(
                        currOwner, 0.0, 5.0, true, RobotParams.ARM_MAX_POWER, visionEvent, 0.0);
                    if (taskParams.objectType == ObjectType.CUBE)
                    {
                        robot.grabber.grabCube(1.0);
                    }
                    else
                    {
                        robot.grabber.grabCone(1.0);
                    }
                    timer.set(1.5, event);
                    sm.waitForSingleEvent(event, State.PREP_FOR_TRAVEL);
                }
                else
                {
                    sm.setState(State.PREP_FOR_TRAVEL);
                }
                break;
            
            case PREP_FOR_TRAVEL:
                // Raise elevator to 5-inch, signal event.
                // Set arm at 5-deg.
                // Wait for event, then goto DONE.
                robot.elevatorPidActuator.setPosition(
                    currOwner, 0.0, 5.0, true, 1.0, event, 0.0);
                robot.armPidActuator.setPosition(
                    currOwner, 0.0, RobotParams.ARM_TRAVEL_POSITION, true, RobotParams.ARM_MAX_POWER, null, 0.0);
                sm.waitForSingleEvent(event, State.DONE);
                break;

            case DONE:
                // Stop task.
                stopAutoTask(true);
                break;
        }
    }   //runTaskState

}   //class TaskAutoPickup
