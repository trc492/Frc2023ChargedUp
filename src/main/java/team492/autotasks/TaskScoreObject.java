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
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
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
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcFrcLib.frclib.FrcPhotonVision.DetectedObject;
import team492.Robot;

/**
 * This class implements auto-assist task to score a cone or cube.
 */
public class TaskScoreObject extends TrcAutoTask<TaskScoreObject.State>
{
    private static final String moduleName = "TaskScoreObject";

    public enum State
    {
        START,
        DETECT_TARGET,
        ALIGN_TO_TARGET,
        PREPARE_TO_SCORE,
        SCORE_OBJECT,
        RESET,
        DONE
    }   //enum State

    public enum ObjectType
    {
        CUBE, 
        CONE
    }   //enum ObjectType

    private static class TaskParams
    {
        ObjectType objectType;
        int scoreLevel;
        double scanPower;
        double scanDuration;
        boolean useVision;

        TaskParams(ObjectType objectType, boolean useVision, int scoreLevel, double scanPower, double scanDuration)
        {
            this.objectType = objectType;
            this.useVision = useVision;
            this.scoreLevel = scoreLevel;
            this.scanPower = scanPower;
            this.scanDuration = scanDuration;
        }   //TaskParams

    }   //class TaskParams

    private final String ownerName;
    private final Robot robot;
    private final TrcDbgTrace msgTracer;
    private final TrcEvent event;
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
    public TaskScoreObject(String ownerName, Robot robot, TrcDbgTrace msgTracer)
    {
        super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK, msgTracer);
        this.ownerName = ownerName;
        this.robot = robot;
        this.msgTracer = msgTracer;
        event = new TrcEvent(moduleName);
    }   //TaskScoreObject

    /**
     * This method ...
     *
     * @param objectType specifies the object type to score (cone or cube).
     * @param useVision specifies true to use vision assist, false otherwise.
     * @param scoreLevel specifies the level to score a cone (not applicable for cube).
     * @param scanPower specifies how fast to scan for target, positive to scan left and negative to scan right.
     * @param scanDuration specifies how long to scan for target in seconds, scan will stop early if detected target.
     * @param event specifies the event to signal when done, can be null if none provided.
     */
    public void autoAssistScoreObject(
        ObjectType objectType, boolean useVision, int scoreLevel, double scanPower, double scanDuration,
        TrcEvent event)
    {
        final String funcName = "autoAssistScoreObject";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(
                funcName, "%s: objectType=%s, scoreLevel=%.2f, scanPower=%.2f, scanDuration=%.3f, event=%s",
                moduleName, objectType, scoreLevel, scanPower, scanDuration, event);
        }

        startAutoTask(State.START, new TaskParams(objectType, useVision, scoreLevel, scanPower, scanDuration), event);
    }   //autoAssistScoreObject
    
    /**
     * This method cancels an in progress auto-assist operation if any.
     */
    public void autoAssistCancel()
    {
        final String funcName = "autoAssistCancel";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(funcName, "%s: Canceling auto-assist score object.", moduleName);
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

        if (ownerName != null)
        {
            if (msgTracer != null)
            {
                TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
                msgTracer.traceInfo(
                    funcName,
                    "%s: Releasing subsystem ownership (currOwner=%s, robotDrive=%s, lift=%s, arm=%s, intake=%s).",
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
        Object params, State state, TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        final String funcName = "runTaskState";
        TaskParams taskParams = (TaskParams) params;
        //
        // Preconditions:
        // Arm is at up position, elevator is at min position, turret is optionally at start scan position.
        //
        switch (state)
        {
            case START:
                sm.setState(taskParams.useVision? State.DETECT_TARGET : State.SCORE_OBJECT);
                break; 

            case DETECT_TARGET:
                detectedTarget = robot.photonVision.getBestDetectedObject();
                //
                // Intentionally falling through to the next state (FIND_POLE).
                //
            case ALIGN_TO_TARGET:
                //Option 1: make input from photonVision the input for the drive pidController like shooter from last year
                    //just in the x direction(rel to robot) first, heading target is + or - 90
                //Option 2: set a relative purePursuit target for how far away the robot should be from the target
                    //this would be apriltagpos - robotpos for x and y, heading would be absolute + or - 90
                //eitherway go to PREPARE_TO_SCORE when done
                TrcPose2D robotPose = robot.photonVision.getRobotFieldPosition(detectedTarget);
                break;

            case PREPARE_TO_SCORE:
                // set elevator, arm, claw, to the proper scoring positions
                break; 

            case SCORE_OBJECT:
                //outtake with the claw then go to RESET state
                break;

            case RESET:
                //retract elevator, arm, everything, back up a tiny bit, go to DONE state  
                break;

            default:
            case DONE:
                stopAutoTask(true);
                break;
        }
    }   //runTaskState

}   //class TaskScoreObject