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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import team492.FrcAuto;
import team492.Robot;
import team492.RobotParams;
import team492.FrcAuto.ObjectType;
import team492.FrcAuto.ScoreLocation;
import team492.vision.PhotonVision.PipelineType;

/**
 * This class implements auto-assist task to score a cone or cube.
 */
public class TaskAutoScore extends TrcAutoTask<TaskAutoScore.State>
{
    private static final String moduleName = "TaskAutoScore";

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

    private static class TaskParams
    {
        ObjectType objectType;
        int scoreLevel;
        boolean useVision;
        ScoreLocation scoreLocation; 
        TaskParams(ObjectType objectType, int scoreLevel,  ScoreLocation scoreLocation, boolean useVision)
        {
            this.objectType = objectType;
            this.scoreLevel = scoreLevel;
            this.useVision = useVision;
            this.scoreLocation = scoreLocation; 
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
    public TaskAutoScore(String ownerName, Robot robot, TrcDbgTrace msgTracer)
    {
        super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK, msgTracer);
        this.ownerName = ownerName;
        this.robot = robot;
        this.msgTracer = msgTracer;
        event = new TrcEvent(moduleName);
    }   //TaskAutoScore

    /**
     * This method starts the auto-assist operation to score an object.
     *
     * @param objectType specifies the object type to score (cone or cube).
     * @param scoreLevel specifies the level to score a cone.
     * @param useVision specifies true to use vision assist, false otherwise.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoAssistScoreObject(
        ObjectType objectType, int scoreLevel, ScoreLocation scoreLocation, boolean useVision, TrcEvent completionEvent)
    {
        final String funcName = "autoAssistScoreObject";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(
                funcName, "%s: objectType=%s, scoreLevel=%d, useVision=%s, event=%s",
                moduleName, objectType, scoreLevel, useVision, completionEvent);
        }

        startAutoTask(State.START, new TaskParams(objectType, scoreLevel, scoreLocation, useVision), completionEvent);
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
                           robot.armPidActuator.acquireExclusiveAccess(ownerName));

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
                    "%s: Releasing subsystem ownership (currOwner=%s, robotDrive=%s, elevator=%s, arm=%s).",
                    moduleName, currOwner, ownershipMgr.getOwner(robot.robotDrive.driveBase),
                    ownershipMgr.getOwner(robot.elevatorPidActuator), ownershipMgr.getOwner(robot.armPidActuator));
            }
            robot.robotDrive.driveBase.releaseExclusiveAccess(currOwner);
            robot.elevatorPidActuator.releaseExclusiveAccess(currOwner);
            robot.armPidActuator.releaseExclusiveAccess(currOwner);
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
                //if using vision, go to DETECT_TARGET state, otherwise go to PREPARE_TO_SCORE
                sm.setState(taskParams.useVision? State.DETECT_TARGET : State.SCORE_OBJECT);
                //
                // Intentionally falling through to the DETECT_TARGET state.
                //
            case DETECT_TARGET:
                robot.photonVision.setPipeline(PipelineType.APRILTAG);
                robot.photonVision.detectBestObject(event, 0.5);
                sm.waitForSingleEvent(event, State.ALIGN_TO_TARGET);
                break;

            case ALIGN_TO_TARGET:
                detectedTarget = robot.photonVision.getLastDetectedBestObject();
                if (detectedTarget == null)
                {
                    // TODO (Code Review): If vision doesn't see the target, what do you do?
                    sm.setState(State.PREPARE_TO_SCORE);
                }
                else
                {
                    robotPose = robot.photonVision.getRobotFieldPosition(detectedTarget);
                    robot.robotDrive.setFieldPosition(robotPose, false);
                    robot.robotDrive.purePursuitDrive.start(
                        currOwner, event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                        getScoringPos(detectedTarget, taskParams.objectType, taskParams.scoreLocation));
                    sm.waitForSingleEvent(event, State.PREPARE_TO_SCORE);
                }
                break;

            case PREPARE_TO_SCORE:
                // set elevator, arm to the proper scoring positions
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

    /**
     * This method returns the absolute field location for the robot to be at to score the game element.
     *
     * @param aprilTagObj specifies the nearest detected AprilTag object.
     * @parm objectType specifies the game element type to score.
     */
    private TrcPose2D getScoringPos(DetectedObject aprilTagObj, ObjectType objectType, ScoreLocation scoreLocation)
    {
        Pose3d aprilTagPos = robot.photonVision.getAprilTagPose(aprilTagObj.target.getFiducialId());
        // Make the robot face the AprilTag.
        double heading = (aprilTagPos.getRotation().getZ() + 180.0) % 360.0; 
        // For now, always score on the junction to the right (for red alliance).
        //x + 22 if red right or blue left
        int xOffset = 0; 
        if(objectType == ObjectType.CONE){
            if((FrcAuto.autoChoices.getAlliance() == Alliance.Red && scoreLocation == ScoreLocation.RIGHT) || 
                FrcAuto.autoChoices.getAlliance() == Alliance.Blue && scoreLocation == ScoreLocation.LEFT){
                xOffset = 22; 
            }
            else{
                xOffset = -22; 
            }
        }
        double x = aprilTagPos.getX() + xOffset;
        double y = aprilTagPos.getY() +
                   (FrcAuto.autoChoices.getAlliance() == Alliance.Blue?
                        RobotParams.STARTPOS_BLUE_Y: RobotParams.STARTPOS_RED_Y);
        return new TrcPose2D(x, y, heading);
    }   //getScoringPos

}   //class TaskAutoScore
