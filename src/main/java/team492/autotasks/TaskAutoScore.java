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
import TrcCommonLib.trclib.TrcTimer;
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
        DRIVE_TO_SCORING_POS,
        SCORE_OBJECT,
        RESET,
        DONE
    }   //enum State

    private static class TaskParams
    {
        ObjectType objectType;
        int scoreLevel;
        ScoreLocation scoreLocation;
        boolean useVision;
        boolean isPreload;

        TaskParams(
            ObjectType objectType, int scoreLevel,  ScoreLocation scoreLocation, boolean useVision, boolean isPreload)
        {
            this.objectType = objectType;
            this.scoreLevel = scoreLevel;
            this.scoreLocation = scoreLocation;
            this.useVision = useVision;
            this.isPreload = isPreload;
        }   //TaskParams
    }   //class TaskParams

    private final String ownerName;
    private final Robot robot;
    private final TrcDbgTrace msgTracer;
    private final TrcEvent event;
    private final TrcEvent visionEvent;
    private final TrcTimer timer;
    private String currOwner = null;

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
        visionEvent = new TrcEvent(moduleName + ".visionEvent");
        timer = new TrcTimer(moduleName);
    }   //TaskAutoScore

    /**
     * This method starts the auto-assist operation to score an object.
     *
     * @param objectType specifies the object type to score (cone or cube).
     * @param scoreLevel specifies the level to score a cone.
     * @param scoreLocation specifies the score location (Left Pole, Shelf, Right Pole).
     * @param useVision specifies true to use vision assist, false otherwise.
     * @param isPreload specifies true to score the preloaded object.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoAssistScoreObject(
        ObjectType objectType, int scoreLevel, ScoreLocation scoreLocation, boolean useVision, boolean isPreload,
        TrcEvent completionEvent)
    {
        final String funcName = "autoAssistScoreObject";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(
                funcName, "%s: objectType=%s, scoreLevel=%d, scoreLocation=%s, useVision=%s, isPreload=%s, event=%s",
                moduleName, objectType, scoreLevel, scoreLocation, useVision, isPreload, completionEvent);
        }

        startAutoTask(
            State.START, new TaskParams(objectType, scoreLevel, scoreLocation, useVision, isPreload), completionEvent);
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
                          robot.robotDrive.driveBase.acquireExclusiveAccess(ownerName) &&
                          robot.elevatorPidActuator.acquireExclusiveAccess(ownerName) &&
                          robot.armPidActuator.acquireExclusiveAccess(ownerName);
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
        TaskParams taskParams = (TaskParams) params;
        // TODO (Code Review): Recommendations
        //  Preconditions:
        //      robot is at position that it can see AprilTag (if using vision) and far enough to deploy elevator and
        //      arm without hitting field elements (Nathan must satisfy these conditions even for scoring preload).
        //  START:
        //      Raise elevator to scoring height, signal elevator event.
        //      Raise arm to scoring angle, signal arm event.
        //      Retract whacker.
        //      if (useVision) call vision to detect target with a timeout, signal vision event.
        //      wait for all events and goto DRIVE_TO_SCORING_POS.
        //  DRIVE_TO_SCORING_POS:
        //      if no position, determine the score position on your own unless its the preload object(just drive straight forward)
        //      use purePursuit to go there (go slowly), goto SCORE_OBJECT.
        //  SCORE_OBJECT:
        //      if object is CUBE, release cube grabber, goto RESET.
        //      if object is CONE, lower arm and release cone with a slight delay, goto RESET.
        //  RESET:
        //      lower elevator to 0 height.
        //      lower arm to TRAVEL_POS.
        //      goto DONE.
        //  DONE:
        //      stop task.
        switch (state)
        {
            case START:
                //determine arm, elevator scoring positions based on scoringLevel and objectType
                double elevatorPos; 
                double armPos;
                if(taskParams.objectType == ObjectType.CONE){
                elevatorPos = RobotParams.elevatorConeScoringPresets[taskParams.scoreLevel];
                armPos = RobotParams.armConeScorePresets[taskParams.scoreLevel];
                }
                else{
                elevatorPos = RobotParams.elevatorCubeScoringPresets[taskParams.scoreLevel];
                armPos = RobotParams.armCubeScorePresets[taskParams.scoreLevel];
                }
                robot.elevatorPidActuator.setPosition(
                    currOwner, 0.0, elevatorPos, true, 1.0, event, 0.0);
                sm.addEvent(event);
                // Assuming the arm is much faster than the elevator.
                robot.armPidActuator.setPosition(currOwner, 0.0, armPos, true, RobotParams.ARM_MAX_POWER, null, 0.0);
                robot.intake.retract(1.0);
                if (taskParams.useVision)
                {
                    robot.photonVision.setPipeline(PipelineType.APRILTAG);
                    robot.photonVision.detectBestObject(visionEvent, RobotParams.VISION_TIMEOUT);
                    sm.addEvent(visionEvent);
                }
                sm.waitForEvents(State.DRIVE_TO_SCORING_POS, 0.0, true);
                break;

            case DRIVE_TO_SCORING_POS:
                TrcPose2D targetPose = null;
                DetectedObject detectedTarget = null; 
                robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.25);
            //if useVision, try to find the target
                if (taskParams.useVision)
                {
                detectedTarget = robot.photonVision.getLastDetectedBestObject();
                }
                //whether detectedTarget was found or not targetPose is getScoringPos(detectedTarget)
                //if detectedTarget is null(vision failed or no vision) getScoringPos() can use odometry to find targetPose
            targetPose = getScoringPos(detectedTarget, taskParams.objectType, taskParams.scoreLocation);
            //if its not the preload object use targetPose to drive to the scoring location
            if (!taskParams.isPreload)
                {
                    robot.robotDrive.purePursuitDrive.start(
                        currOwner, event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                        targetPose);
                }
                //for preload we only want to drive forward before scoring, need to tune the y value
                else{
                robot.robotDrive.purePursuitDrive.start(
                    currOwner, event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                    new TrcPose2D(0.0, 20.0, 0.0));
                }

                sm.waitForSingleEvent(event, State.SCORE_OBJECT);
                break;

            case SCORE_OBJECT:
                robot.robotDrive.purePursuitDrive.setMoveOutputLimit(1.0);
                if (taskParams.objectType == ObjectType.CUBE)
                {
                    robot.grabber.releaseAll();
                }
                else
                {
                    // Move the arm down to cap the pole, then release the cone with a slight delay.
                    robot.armPidActuator.setPosition(
                        currOwner, 0.0, RobotParams.ARM_PICKUP_POSITION, true, RobotParams.ARM_MAX_POWER, visionEvent, 0);
                    robot.grabber.releaseCube();
                    robot.grabber.releaseCone(0.2);
                }
                //outtake with the claw then go to RESET state
                timer.set(0.2, event);
                sm.waitForSingleEvent(event, State.RESET);
                break;

            case RESET:
                // Retract elevator, arm, everything, back up a tiny bit, go to DONE state.
                robot.elevatorPidActuator.setPosition(
                    currOwner, 0.0, RobotParams.ELEVATOR_MIN_POS, true, 1.0, event, 0.0);
                robot.armPidActuator.setPosition(
                    currOwner, 0.0, RobotParams.ARM_TRAVEL_POSITION, true, RobotParams.ARM_MAX_POWER, null, 0.0);
                robot.robotDrive.purePursuitDrive.start(
                    currOwner, null, 0.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                    new TrcPose2D(0.0, -20.0, 0.0));
                sm.waitForSingleEvent(event, State.DONE);
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
     * @param aprilTagObj specifies the nearest detected AprilTag object, can be null if vision not found anything.
     * @param objectType specifies the game element type to score.
     * @param scoreLocation specifies the scoring location (shelf, left pole or right pole).
     */
    private TrcPose2D getScoringPos(DetectedObject aprilTagObj, ObjectType objectType, ScoreLocation scoreLocation)
    {
        double scoringPosX;

        if (aprilTagObj != null)
        {
            Pose3d aprilTagPos = robot.photonVision.getAprilTagPose(aprilTagObj.target.getFiducialId());
            scoringPosX = aprilTagPos.getX();
        } 
        else 
        {  
            TrcPose2D robotPos = robot.robotDrive.driveBase.getFieldPosition();
            double minXDist = Math.abs(robotPos.x - RobotParams.startPosX[0]);
            int minXDistIndex = 0;
            // Find the closest StartPos from the robot's current location.
            for (int i = 1; i < RobotParams.startPosX.length; i++)
            {
                double xDist = Math.abs(robotPos.x - RobotParams.startPosX[i]);
                if (xDist < minXDist)
                {
                    minXDist = xDist;
                    minXDistIndex = i;
                }
            }
            scoringPosX = RobotParams.startPosX[minXDistIndex];
        }

        if (objectType == ObjectType.CONE)
        {
            // Pole is 22 inches to the left or right of the StartPos.
            if (FrcAuto.autoChoices.getAlliance() == Alliance.Blue && scoreLocation == ScoreLocation.LEFT ||
                FrcAuto.autoChoices.getAlliance() == Alliance.Red && scoreLocation == ScoreLocation.RIGHT)
            {
                scoringPosX += 22.0;
            }
            else
            {
                scoringPosX -= 22.0;
            }
        }

        return new TrcPose2D(
            scoringPosX,
            FrcAuto.autoChoices.getAlliance() == Alliance.Blue?
                RobotParams.STARTPOS_BLUE_Y + 24.0: RobotParams.STARTPOS_RED_Y - 24.0,
            FrcAuto.autoChoices.getAlliance() == Alliance.Blue? 180.0: 0.0);
    }   //getScoringPos
 
 }   //class TaskAutoScore