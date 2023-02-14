/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
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

package team492;

import TrcCommonLib.trclib.TrcAutoTask;
import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcOwnershipMgr;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcFrcLib.frclib.FrcPhotonVision.DetectedObject;

/**
 * This class implements auto-assist task to score a cone.
 */
public class TaskScoreObject extends TrcAutoTask<TaskScoreObject.State>
{
    private static final String moduleName = "TaskScoreCone";
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
    public enum ObjectType{
        CUBE, 
        CONE
    }

    private static class TaskParams
    {

        ObjectType objectType;
        int scoreLevel;
        double scanPower;
        double scanDuration;
        boolean useVision;

        TaskParams(
            ObjectType objectType, boolean useVision, int scoreLevel, double scanPower,
            double scanDuration)
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
    }   //TaskScoreCone

    /**
     * This method turns the turret to the start location, raises the elevator to the score height and starts scanning
     * for the pole by slowly turning the turret for a given maximum duration. If the pole is detected by the sensor,
     * it will stop the turret, extend the arm on top of the pole, lower the elevator to cap the pole, release the cone
     * and retract the arm and elevator.
     *
     * @param startTarget specifies the absolute start turret position to go to before doing the slow scan.
     * @param startPowerLimit specifies the turret power limit going to the startTarget.
     * @param expectedTarget specifies the expect turret position where the target should be.
     * @param scoreHeight specifies the elevator height to score the cone.
     * @param scanPower specifies how fast to scan for target, positive to scan left and negative to scan right.
     * @param scanDuration specifies how long to scan for target in seconds, scan will stop early if detected target.
     * @param event specifies the event to signal when done, can be null if none provided.
     */
    public void autoAssistScoreCone(
        ObjectType objectType, boolean useVision, int scoreLevel, double scanPower,
        double scanDuration, TrcEvent event)
    {
        final String funcName = "autoAssistScoreCone";
        if (msgTracer != null)
        {
            
            msgTracer.traceInfo(
                funcName,
                "%s: objectType=%s, scoreLevel=%.2f, scanPower=%.2f, " +
                "scanDuration=%.3f, event=%s",
                moduleName, objectType, scoreLevel, scanPower, scanDuration, event);
        }

        startAutoTask(
            State.START, new TaskParams(
                objectType, useVision, scoreLevel, scanPower, scanDuration), event);
    }   //autoAssistScoreCone
    
    /**
     * This method cancels an in progress auto-assist operation if any.
     */
    public void autoAssistCancel()
    {
        final String funcName = "autoAssistCancel";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(funcName, "%s: Canceling auto-assist score cone.", moduleName);
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
                          (robot.intake.acquireExclusiveAccess(ownerName) &&
                           robot.lift.acquireExclusiveAccess(ownerName) &&
                           robot.arm.acquireExclusiveAccess(ownerName) &&
                           robot.robotDrive.driveBase.acquireExclusiveAccess(ownerName));

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
                // msgTracer.traceInfo(
                //     funcName, "%s: Releasing subsystem ownership (currOwner=%s, intake=%s, robotDrive=%s, grabber=%s, arm=%s).",
                //     moduleName, currOwner, ownershipMgr.getOwner(robot.intake),
                //     ownershipMgr.getOwner(robot.robotDrive), ownershipMgr.getOwner(robot.ownershipMgr.getOwner(robot.grabber));
            }
            robot.intake.releaseExclusiveAccess(currOwner);
            robot.robotDrive.driveBase.releaseExclusiveAccess(currOwner);
            robot.arm.releaseExclusiveAccess(currOwner);
            robot.lift.releaseExclusiveAccess(currOwner);
            currOwner = null;
        }
    }   //releaseSubsystemsOwnership

    /**
     * This method is called by the super class to stop all the subsystems.
     */
    @Override
    protected void stopSubsystems()
    {
        //todo write this
        // robot.intake.cancel(currOwner);
        // robot.robotDrive.cancel(currOwner);
        // robot.arm.cancel(currOwner);
    }   //stopSubsystems.

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

}   //class TaskScoreCone