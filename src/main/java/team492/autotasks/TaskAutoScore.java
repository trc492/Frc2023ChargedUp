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
import TrcCommonLib.trclib.TrcRobot.RunMode;
import TrcFrcLib.frclib.FrcPhotonVision.DetectedObject;
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
        POSITION_FOR_SCORE,
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
        boolean positionOnly; 

        TaskParams(
            ObjectType objectType, int scoreLevel,  ScoreLocation scoreLocation, boolean useVision, boolean positionOnly)
        {
            this.objectType = objectType;
            this.scoreLevel = scoreLevel;
            this.scoreLocation = scoreLocation;
            this.useVision = useVision;
            this.positionOnly = positionOnly; 
        }   //TaskParams
    }   //class TaskParams

    private final String ownerName;
    private final Robot robot;
    private final TrcDbgTrace msgTracer;
    private final TrcEvent elevatorEvent;
    private final TrcEvent armEvent;
    private final TrcEvent wristEvent;
    private final TrcEvent visionEvent;
    private final TrcEvent driveEvent;
    private final TrcEvent event;
    private Alliance alliance;
    private String currOwner = null;
    private String driveOwner = null;

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

        elevatorEvent = new TrcEvent(moduleName + ".elevatorEvent");
        armEvent = new TrcEvent(moduleName + ".armEvent");
        wristEvent = new TrcEvent(moduleName + ".wristEvent");
        visionEvent = new TrcEvent(moduleName + ".visionEvent");
        driveEvent = new TrcEvent(moduleName + ".driveEvent");
        event = new TrcEvent(moduleName);
    }   //TaskAutoScore

    /**
     * This method starts the auto-assist operation to score an object.
     *
     * @param objectType specifies the object type to score (cone or cube).
     * @param scoreLevel specifies the level to score a cone.
     * @param scoreLocation specifies the score location (Left Pole, Shelf, Right Pole).
     * @param useVision specifies true to use vision assist, false otherwise.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoAssistScoreObject(
        ObjectType objectType, int scoreLevel, ScoreLocation scoreLocation, boolean useVision,
        TrcEvent completionEvent, boolean positionOnly)
    {
        final String funcName = "autoAssistScoreObject";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(
                funcName, "%s: objectType=%s, scoreLevel=%d, scoreLocation=%s, useVision=%s, event=%s",
                moduleName, objectType, scoreLevel, scoreLocation, useVision, completionEvent);
        }

        startAutoTask(
            State.START, new TaskParams(objectType, scoreLevel, scoreLocation, useVision, positionOnly), completionEvent);
    }   //autoAssistScoreObject

    //only positions, arm, elevator, grabber for scoring at the right level, no vision
    //ScoreLocation will not be used
    public void autoAssistScoreConePositionOnly(int scoreLevel){

        //can't use this yet: 
        //preset is meant to raise everything to position but stopAutoTask will cause all power to be cut
        // autoAssistScoreObject(ObjectType.CONE, scoreLevel, ScoreLocation.LEFT, false, null, true);
        if(scoreLevel != 0 ){
            double elevatorPos = RobotParams.elevatorConeScorePresets[scoreLevel];
            double armPos = RobotParams.armConeScorePresets[scoreLevel];
            double wristPos = RobotParams.wristConeScorePresets[scoreLevel];
            robot.elevatorPidActuator.setPosition(
                currOwner, 0.0, elevatorPos, true, 1.0, elevatorEvent, 0.7);
            robot.armPidActuator.setPosition(
                currOwner, 0.0, armPos, true, RobotParams.ARM_MAX_POWER, armEvent, 0.7);
            robot.wristPidActuator.setPosition(
                currOwner, 0.0, wristPos, true, RobotParams.WRIST_MAX_POWER, wristEvent, 0.7);
        }
        else if (robot.weedWhacker != null)
        {
            robot.weedWhacker.extend();
            robot.weedWhacker.setPower(currOwner, 0.5, RobotParams.INTAKE_SPIT_POWER, 0.3, null);
        }


    }
    public void autoAssistScoreCubePositionOnly(int scoreLevel){
        //same logic as above 
        // autoAssistScoreObject(ObjectType.CONE, scoreLevel, ScoreLocation.LEFT, false, null, true);
        //for now assumes level is not 0 
        if(scoreLevel != 0){
            double elevatorPos = RobotParams.elevatorCubeScorePresets[scoreLevel];
            double armPos = RobotParams.armCubeScorePresets[scoreLevel];
            double wristPos = RobotParams.wristCubeScorePresets[scoreLevel];
            robot.elevatorPidActuator.setPosition(
                currOwner, 0.0, elevatorPos, true, 1.0, elevatorEvent, 0.7);
            robot.armPidActuator.setPosition(
                currOwner, 0.0, armPos, true, RobotParams.ARM_MAX_POWER, armEvent, 0.7);
            robot.wristPidActuator.setPosition(
                currOwner, 0.0, wristPos, true, RobotParams.WRIST_MAX_POWER, wristEvent, 0.7);
        }
        else if (robot.weedWhacker != null)
        {
            robot.weedWhacker.extend();
            robot.weedWhacker.setPower(currOwner, 0.5, RobotParams.INTAKE_SPIT_POWER, 0.3, null);
        }
    }
    /**
     * This method starts the auto-assist operation to score an object.
     *
     * @param scoreLevel specifies the level to score a cone.
     * @param useVision specifies true to use vision assist, false otherwise.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoAssistScoreCube(int scoreLevel, boolean useVision,TrcEvent completionEvent)
    {
        autoAssistScoreObject(ObjectType.CUBE, scoreLevel, ScoreLocation.MIDDLE, useVision, completionEvent, false);
    }   //autoAssistScoreCube

    /**
     * This method starts the auto-assist operation to score an object.
     *
     * @param scoreLevel specifies the level to score a cone.
     * @param scoreLocation specifies the score location (Left Pole, Shelf, Right Pole).
     * @param useVision specifies true to use vision assist, false otherwise.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoAssistScoreCone(int scoreLevel, ScoreLocation scoreLocation, boolean useVision,TrcEvent completionEvent)
    {
        if (scoreLocation == ScoreLocation.MIDDLE)
        {
            throw new IllegalArgumentException("Cone cannot be scored in the middle.");
        }

        autoAssistScoreObject(ObjectType.CONE, scoreLevel, scoreLocation, useVision, completionEvent, false);
    }   //autoAssistScoreCone

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
                          robot.elevatorPidActuator.acquireExclusiveAccess(ownerName) &&
                          robot.armPidActuator.acquireExclusiveAccess(ownerName) &&
                          robot.wristPidActuator.acquireExclusiveAccess(ownerName) &&
                          robot.intake.acquireExclusiveAccess(ownerName);
        // Don't acquire drive base ownership globally. Acquire it only if we need to drive.
        // For example, we only need to drive if we are using vision to approach the score location.
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

        if (ownerName != null)
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
        Object params, State state, TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        TaskParams taskParams = (TaskParams) params;
        //  Preconditions:
        //      Robot is at position that it can see AprilTag (if using vision) and far enough to deploy elevator and
        //      arm without hitting field elements.
        switch (state)
        {
            //TODO: Test manually if we can extend everything with bumpers touching grid, 
            //if we can, new sequence of states to be: 
            //START: retract everything (Turtle Mode), use vision to detect apriltag
            //DRIVE_TO_SCORE: drive so bumpers touch the correct grid 
            //POSITION_TO_SCORE: basically what we have in START state 
            //SCORE

            case START:
                double elevatorPos, armPos, wristPos;
                // Determine arm, elevator, wrist positions based on scoringLevel and objectType.
                alliance = FrcAuto.autoChoices.getAlliance();
                if (taskParams.objectType == ObjectType.CONE)
                {
                    elevatorPos = RobotParams.elevatorConeScorePresets[taskParams.scoreLevel];
                    armPos = RobotParams.armConeScorePresets[taskParams.scoreLevel];
                    wristPos = RobotParams.wristConeScorePresets[taskParams.scoreLevel];
                }
                else
                {
                    elevatorPos = RobotParams.elevatorCubeScorePresets[taskParams.scoreLevel];
                    armPos = RobotParams.armCubeScorePresets[taskParams.scoreLevel];
                    wristPos = RobotParams.wristCubeScorePresets[taskParams.scoreLevel];
                }

                if (taskParams.scoreLevel > 0)
                {
                    robot.elevatorPidActuator.setPosition(
                        currOwner, 0.0, elevatorPos, true, 1.0, elevatorEvent, 0.7);
                    sm.addEvent(elevatorEvent);
                    robot.armPidActuator.setPosition(
                        currOwner, 0.0, armPos, true, RobotParams.ARM_MAX_POWER, armEvent, 0.7);
                    sm.addEvent(armEvent);
                    robot.wristPidActuator.setPosition(
                        currOwner, 0.0, wristPos, true, RobotParams.WRIST_MAX_POWER, wristEvent, 0.7);
                    sm.addEvent(wristEvent);

                    if (robot.weedWhacker != null)
                    {
                        robot.weedWhacker.retract();
                    }
                }
                else if (robot.weedWhacker != null)
                {
                    robot.weedWhacker.extend();
                }

                if (taskParams.useVision)
                {
                    robot.photonVision.setPipeline(PipelineType.APRILTAG);
                    robot.photonVision.detectBestObject(visionEvent, RobotParams.VISION_TIMEOUT);
                    sm.addEvent(visionEvent);
                }
                if(!taskParams.positionOnly){
                    sm.waitForEvents(taskParams.useVision? State.DRIVE_TO_SCORING_POS: State.SCORE_OBJECT, true);
                }
                else{
                    sm.waitForEvents(State.DONE); 
                }
                break;

            case DRIVE_TO_SCORING_POS:
                // If useVision, set robot's field position using detected target info.
                // Determine scoring position either by vision or drive base odometry.
                // Drive to the scoring position slowly, then goto SCORE_OBJECT.
                if (acquireDriveBaseOwnership())
                {
                    DetectedObject detectedTarget = robot.photonVision.getLastDetectedBestObject();
                    if (detectedTarget != null)
                    {
                        TrcPose2D robotPose = robot.photonVision.getRobotFieldPosition(detectedTarget);
                        robot.robotDrive.driveBase.setFieldPosition(robotPose);
                        robot.globalTracer.traceInfo(
                            moduleName, "Detected %s: robotPose=%s", robot.photonVision.getPipeline(), robotPose);
                    }
                    // getScoringPos will return the scoring position either from vision detected target or from drive
                    // base odometry if not using vision or vision did not detect target.
                    TrcPose2D targetPose = getScoringPos(detectedTarget, taskParams.objectType, taskParams.scoreLocation);
                    robot.globalTracer.traceInfo(moduleName, "TargetPose=%s", targetPose);

                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.5);
                    robot.robotDrive.purePursuitDrive.setMsgTracer(msgTracer, true, true);
                    robot.robotDrive.purePursuitDrive.start(
                        currOwner, driveEvent, 4.0, robot.robotDrive.driveBase.getFieldPosition(), false, targetPose);
                    sm.waitForSingleEvent(driveEvent, State.SCORE_OBJECT);
                }
                break;


            case SCORE_OBJECT:
                robot.robotDrive.purePursuitDrive.setMsgTracer(msgTracer, false, false);
                if (taskParams.objectType == ObjectType.CUBE && taskParams.scoreLevel == 0 && robot.weedWhacker != null)
                {
                    // Scoring cube at ground level, use the weedwhacker.
                    robot.weedWhacker.setPower(currOwner, 0.0, RobotParams.WEEDWHACKER_SPIT_POWER, 0.5, event);
                }
                else
                {
                    robot.intake.autoAssistIntake(currOwner, 0.0, RobotParams.INTAKE_SPIT_POWER, 0.0, event, 0.0);
                }
                break;

            case RESET:
                // Back up 2 feet.
                // Lower elevator to 0 height after a delay to make sure we don't hit anything.
                // Lower arm to TRAVEL_POS after a delay to make sure we don't hit anything.
                // goto DONE.
                robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.25);
                // TODO: Tune delays & timeout
                robot.robotDrive.purePursuitDrive.start(
                    currOwner, driveEvent, 0.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                    new TrcPose2D(0.0, -24.0, 0.0));
                robot.turtleMode(currOwner);
                sm.waitForSingleEvent(driveEvent, State.DONE, 2.0);
                break; 

            default:
            case DONE:
                // Stop task.
                robot.globalTracer.traceInfo(
                    "AutoScore Done", "Robot thinks it is at:%s",
                    robot.robotDrive.driveBase.getFieldPosition());
                robot.robotDrive.purePursuitDrive.setMoveOutputLimit(RobotParams.PPD_MOVE_DEF_OUTPUT_LIMIT);
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

   /**
     * This method returns the absolute field location for the robot to be at to score the game element.
     *
     * @param aprilTagObj specifies the nearest detected AprilTag object, can be null if vision not found anything.
     * @param objectType specifies the game element type to score.
     * @param scoreLocation specifies the scoring location (shelf, left pole or right pole).
     * @return absolute field location for the robot to score the object.
     */
    private TrcPose2D getScoringPos(DetectedObject aprilTagObj, ObjectType objectType, ScoreLocation scoreLocation)
    {
        double scoringPosX;

        if (aprilTagObj != null)
        {
            TrcPose2D aprilTagPos = DetectedObject.pose3dToTrcPose2D(
                robot.photonVision.getAprilTagPose(aprilTagObj.target.getFiducialId()));
            scoringPosX = aprilTagPos.x;
        }
        else
        {
            // We did not use vision or vision did not find AprilTag, use odometry position and find the nearest
            // AprilTag.
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
            // Pole is some distance offset to the left or right of the StartPos.
            if (alliance == Alliance.Blue && scoreLocation == ScoreLocation.LEFT ||
                alliance == Alliance.Red && scoreLocation == ScoreLocation.RIGHT)
            {
                scoringPosX += 22.0;
            }
            else
            {
                scoringPosX -= 18.0;
            }
        }

        if (robot.getCurrentRunMode() == RunMode.TELEOP_MODE)
        {
            return new TrcPose2D(
                scoringPosX,
                alliance == Alliance.Blue? RobotParams.STARTPOS_BLUE_Y + 4.0: RobotParams.STARTPOS_RED_Y - 4.0,
                alliance == Alliance.Blue? 180.0: 0.0);
        }
        else
        {
            return new TrcPose2D(
                scoringPosX,
                alliance == Alliance.Blue? RobotParams.STARTPOS_BLUE_Y - 6.0: RobotParams.STARTPOS_RED_Y + 6.0,
                alliance == Alliance.Blue? 180.0: 0.0);
        }
    }   //getScoringPos
 
 }   //class TaskAutoScore
 