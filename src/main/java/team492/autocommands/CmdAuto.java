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

 package team492.autocommands;

import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTimer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import team492.FrcAuto;
import team492.Robot;
import team492.RobotParams;
import team492.FrcAuto.ObjectType;
import team492.FrcAuto.ScoreLocation;

public class CmdAuto implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAuto";

    private enum State
    {
        START,
        SCORE_GAME_PIECE,
        START_DELAY,
        GO_TO_GAME_PIECE,
        PICKUP_GAME_PIECE,
        GO_TO_SCORE_POSITION,
        GO_TO_CHARGING_STATION,
        AUTO_BALANCE,
        DONE
    }   //enum State


    private final Robot robot;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;

    private ObjectType loadedObjType;
    private int scoreLevel;
    private ScoreLocation scoreLocation;
    private boolean useVision;
    private boolean doAutoBalance;
    private boolean getSecondPiece;
    //getSecondPiece && doAutoBalance: after the start delay, we go for a second piece, score it, then park
    //!getSecondPiece && doAutoBalance: after the start delay, we go out of the community and then we go to park
    //getSecondPiece && !doAutoBalance: after the start delay, we go for a second piece, score it, then go for a third piece
    //!getSecondPiece && !doAutoBalance: after the start delay, we stop
    private int piecesScored = 0;
    // private ScoreLocation scoreLocation;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     */
    public CmdAuto(Robot robot)
    {
        robot.globalTracer.traceInfo(moduleName, ">>> robot=%s, choices=%s", robot, FrcAuto.autoChoices);

        this.robot = robot;
        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START);
    }   //CmdAuto

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    /**
     * This method checks if the current RobotCommand  is running.
     *
     * @return true if the command is running, false otherwise.
     */
    @Override
    public boolean isActive()
    {
        return sm.isEnabled();
    }   //isActive

    /**
     * This method cancels the command if it is active.
     */
    @Override
    public void cancel()
    {
        robot.autoScoreTask.autoAssistCancel();
        robot.autoPickupTask.autoAssistCancel();
        robot.autoBalanceTask.autoAssistCancel();
        robot.robotDrive.purePursuitDrive.setMoveOutputLimit(RobotParams.PPD_MOVE_DEF_OUTPUT_LIMIT);
        robot.robotDrive.cancel();
        sm.stop();
    }   //cancel

    /**
     * This method must be called periodically by the caller to drive the command sequence forward.
     *
     * @param elapsedTime specifies the elapsed time in seconds since the start of the robot mode.
     * @return true if the command sequence is completed, false otherwise.
     */
    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        State state = sm.checkReadyAndGetState();

        if (state == null)
        {
            robot.dashboard.displayPrintf(8, "State: disabled or waiting (nextState=%s)...", sm.getNextState());
        }
        else
        {
            State nextState;

            robot.dashboard.displayPrintf(8, "State: %s", state);
            switch (state)
            {
                // TODO (Code Review): Eventually, need to change the paths according to the starting position.
                // StartPos1: Go to the path by the field rail side to fetch 2nd game piece.
                // StartPos2: Go to balance on Charging Station after scoring preloaded piece.
                // StartPos3: Go to the path by the substation side to fetch 2nd game piece.
                // Also, need to add code to check match time in order to determine if we have enough time
                // to fetch 2nd piece. If not, check if we can go balance.
                case START:
                    // startPos = FrcAuto.autoChoices.getStartPos();   // 0, 1, or 2.
                    loadedObjType = ObjectType.CUBE;
                    scoreLevel = FrcAuto.autoChoices.getScoreLevel();
                    scoreLocation = FrcAuto.autoChoices.getScoreLocation();
                    useVision = FrcAuto.autoChoices.getUseVision();
                    doAutoBalance = FrcAuto.autoChoices.getDoAutoBalance();
                    getSecondPiece = FrcAuto.autoChoices.getGetSecondPiece();
                    // Set robot's start position according to autoChoices.
                    robot.robotDrive.setFieldPosition(null, false);

                    // Back up a little slowly so that deploying elevator and arm won't hit any field elements.
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.25);
                    robot.robotDrive.purePursuitDrive.start(
                        event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(0.0, -24.0, 0.0));
                    // Raise elevator a little to let the arm out.
                    robot.elevatorPidActuator.setPosition(
                        RobotParams.ELEVATOR_SAFE_HEIGHT, true, 1.0, null, 0.5);
                    robot.armPidActuator.setPosition(
                        0.5, RobotParams.ARM_TRAVEL_POSITION, true, RobotParams.ARM_MAX_POWER, null, 0.0);
                    sm.waitForSingleEvent(event, State.SCORE_GAME_PIECE);
                    break;

                case SCORE_GAME_PIECE:
                    //  Preconditions:
                    //      Robot is at position that it can see AprilTag (if using vision) and far enough to deploy elevator and
                    //      arm without hitting field elements.
                    if (piecesScored == 0)
                    {
                        // TODO (Code Review): Samuel said we should get out of the community instead of doing delay. Please rework.
                        nextState = State.START_DELAY;
                        robot.autoScoreTask.autoAssistScoreObject(
                            loadedObjType, scoreLevel, scoreLocation, false,  event);
                    }
                    else if (piecesScored == 1 && !doAutoBalance)
                    {
                        // Scoring second game piece, going for third next.
                        nextState = State.GO_TO_GAME_PIECE;
                        // Code Review: what is the loadedObjType for the 2nd piece? Who sets scoreLocation?
                        robot.autoScoreTask.autoAssistScoreObject(
                            loadedObjType, scoreLevel, ScoreLocation.LEFT, useVision, event);
                    }
                    else
                    {
                        // Scoring second game piece, going to the charging station next.
                        nextState = State.GO_TO_CHARGING_STATION;
                        robot.autoScoreTask.autoAssistScoreObject(
                            loadedObjType, scoreLevel, ScoreLocation.RIGHT, useVision, event);
                    }
                    sm.waitForSingleEvent(event, nextState);
                    piecesScored++;
                    break;

                case START_DELAY:
                    if (getSecondPiece)
                    {
                        nextState = State.GO_TO_GAME_PIECE;
                    }
                    else if (doAutoBalance)
                    {
                        nextState = State.GO_TO_CHARGING_STATION;
                    }
                    else
                    {
                        nextState = State.DONE;
                    }
                    double startDelay = FrcAuto.autoChoices.getStartDelay();
                    if (startDelay == 0.0)
                    {
                        sm.setState(nextState);
                    }
                    else
                    {
                        sm.waitForSingleEvent(event, nextState);
                        timer.set(startDelay, event);
                    }
                    break;

                case GO_TO_GAME_PIECE:
                    // TODO (Code Review): Please use the dimensions defined in RobotParams.
                    // Drives to a few feet (3ft from the center of the robot to the ball) behind the game piece we
                    // want to pick up, determining the location by how many pieces we have already scored
                    if (piecesScored == 1)
                    {
                        // We are going for the second game piece.
                        // drives to 3ft behind the right most piece from the init scoring pos
                        if (FrcAuto.autoChoices.getAlliance() == Alliance.Blue)
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                new TrcPose2D(RobotParams.CENTER_BETWEEN_CHARGING_STATION_AND_FIELD_EDGE_X, RobotParams.CHARGING_STATION_CENTER_BLUE_Y, 180.0),
                                new TrcPose2D(RobotParams.GAME_PIECE_1_X, RobotParams.GAME_PIECE_BLUE_Y - 36, 0.0));
                        }
                        else
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                new TrcPose2D(RobotParams.CENTER_BETWEEN_CHARGING_STATION_AND_FIELD_EDGE_X, 564.0, 0.0),
                                new TrcPose2D(RobotParams.GAME_PIECE_1_X, RobotParams.GAME_PIECE_RED_Y + 36, 180.0));
                        }
                    }
                    else if (piecesScored == 2)
                    {
                        // We are going for the third game piece.
                        // drives to the location of the second right most piece
                        if (FrcAuto.autoChoices.getAlliance() == Alliance.Blue)
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, 2.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                new TrcPose2D(RobotParams.CENTER_BETWEEN_CHARGING_STATION_AND_FIELD_EDGE_X, 85.0, 0.0),
                                new TrcPose2D(RobotParams.CENTER_BETWEEN_CHARGING_STATION_AND_FIELD_EDGE_X, 220.0, 0.0),
                                new TrcPose2D(RobotParams.GAME_PIECE_2_X, RobotParams.GAME_PIECE_BLUE_Y - 36, 0.0));
                        }
                        else
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, 2.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                new TrcPose2D(RobotParams.CENTER_BETWEEN_CHARGING_STATION_AND_FIELD_EDGE_X, 564.0, 180.0),
                                new TrcPose2D(RobotParams.CENTER_BETWEEN_CHARGING_STATION_AND_FIELD_EDGE_X, 429.0, 180.0),
                                new TrcPose2D(RobotParams.GAME_PIECE_2_X, RobotParams.GAME_PIECE_RED_Y + 36, 180.0));
                        }
                    }
                    sm.waitForSingleEvent(event, State.PICKUP_GAME_PIECE);
                    break;
                    
                case PICKUP_GAME_PIECE:
                    // Drives forward while running intake until it picks up a game piece, the precondition being that
                    // we already at the correct location
                    sm.waitForSingleEvent(event, State.GO_TO_SCORE_POSITION);
                    // Second game piece will be a cube, third game piece will be a cone.
                    robot.autoPickupTask.autoAssistPickup(
                        //piecesScored == 1? ObjectType.CUBE: Right now, we are assuming that we can pick up a cone reliably
                        ObjectType.CONE, useVision, event);
                    //if (piecesScored == 1) {
                    //    loadedObjType = ObjectType.CUBE;
                    //}
                    //else {
                    loadedObjType = ObjectType.CONE;
                    //}
                    //we dont need the folowing lines, keeping it just in case
                    // robot.robotDrive.purePursuitDrive.start(
                    //     null, 2.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                    //     new TrcPose2D(0, 50, 0));
                    break;

                case GO_TO_SCORE_POSITION:
                    // TODO (Code Review): Please use the dimensions defined in RobotParams.
                    // Drives to the scoring position, determining the location based on how many pieces we have
                    // already scored
                    if (piecesScored == 1)
                    {
                        //drives to the score precondition position from the right most game piece
                        if (FrcAuto.autoChoices.getAlliance() == Alliance.Blue)
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, 2.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                new TrcPose2D(RobotParams.CENTER_BETWEEN_CHARGING_STATION_AND_FIELD_EDGE_X, 85.0, 180.0));
                        }
                        else
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, 2.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                new TrcPose2D(RobotParams.CENTER_BETWEEN_CHARGING_STATION_AND_FIELD_EDGE_X, 564.0, 0.0));
                        }
                    }
                    else if (piecesScored == 2)
                    {
                        // drives to the score precondition position from the second right most game piece
                        if (FrcAuto.autoChoices.getAlliance() == Alliance.Blue)
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, 2.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                new TrcPose2D(RobotParams.CENTER_BETWEEN_CHARGING_STATION_AND_FIELD_EDGE_X, 220.0, 180.0),
                                new TrcPose2D(RobotParams.CENTER_BETWEEN_CHARGING_STATION_AND_FIELD_EDGE_X, 85.0, 180.0));
                        }
                        else
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, 2.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                new TrcPose2D(RobotParams.CENTER_BETWEEN_CHARGING_STATION_AND_FIELD_EDGE_X, 429, 0.0),
                                new TrcPose2D(RobotParams.CENTER_BETWEEN_CHARGING_STATION_AND_FIELD_EDGE_X, 564.0, 0.0));
                        }
                    }
                    sm.waitForSingleEvent(event, State.SCORE_GAME_PIECE);
                    break;

                case GO_TO_CHARGING_STATION:
                    // Drives to just behind and then onto the charging station platform from the scoring position
                    if (FrcAuto.autoChoices.getAlliance() == Alliance.Blue)
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, 2.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            new TrcPose2D(-107.0, 85.0, 270),
                            new TrcPose2D(-107.0, 145.0, 270));
                    }
                    else
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, 2.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            new TrcPose2D(-107.0, 550.0, 90),
                            new TrcPose2D(-107.0, 490.0, 90));
                    }
                    sm.waitForSingleEvent(event, State.AUTO_BALANCE);
                    break;

                case AUTO_BALANCE:
                    robot.autoBalanceTask.autoAssistBalance(event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    cancel();
                    break;
            }

            robot.globalTracer.traceStateInfo(
                sm.toString(), state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                robot.robotDrive.purePursuitDrive, null);
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdAuto
