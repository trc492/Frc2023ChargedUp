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
import team492.Robot;
import team492.RobotParams;
import team492.FrcAuto.BalanceInitSide;
import team492.FrcAuto.ObjectType;
import team492.FrcAuto.ScoreLocation;

public class CmdAutoStartPos1Or3 implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoStartPos1Or3";

    private enum State
    {
        START,
        BACK_UP,
        UNTUCK_ARM,
        SCORE,
        GET_SECOND,
        DRIVE_TO_SCORE,
        DRIVE_TO_BALANCE,
        BALANCE,
        EXIT_COMMUNITY,
        DONE
    }   //enum State


    private final Robot robot;
    private final TrcTimer timer;
    private final TrcEvent driveEvent;
    private final TrcEvent elevatorEvent;
    private final TrcEvent autoAssistEvent;
    private final TrcEvent tiltEvent;
    private final TrcEvent distanceEvent;
    private final TrcEvent intakeEvent;    private final TrcStateMachine<State> sm;

    private Alliance alliance = Alliance.Blue;
    /*
     * 0: Blue right, red left
     * 2: blue left, red right
     */
    private int startPos = 2;
    // private ObjectType loadedObjType;
    private int scoreLevel = 0;
    private ScoreLocation scoreLocation = ScoreLocation.MIDDLE;
    private boolean useVision = false;
    private boolean scorePreload = true;
    private boolean doAutoBalance = false;
    private boolean scoreSecondPiece = true;
    //scoreSecondPiece && doAutoBalance: after the start delay, we go for a second piece, score it, then park
    //!scoreSecondPiece && doAutoBalance: after the start delay, we go out of the community, pickup a second piece, and then park
    //scoreSecondPiece && !doAutoBalance: after the start delay, we go for a second piece and score it
    //!scoreSecondPiece && !doAutoBalance: after the start delay, we stop
    private int piecesScored = 0;
    private boolean untuck = false;
    private double xOffset = 0.0;
    // private ScoreLocation scoreLocation;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     */
    public CmdAutoStartPos1Or3(Robot robot)
    {
        this.robot = robot;
        timer = new TrcTimer(moduleName);
        driveEvent = new TrcEvent(moduleName + ".driveEvent");
        elevatorEvent = new TrcEvent(moduleName + ".elevatorEvent");
        autoAssistEvent = new TrcEvent(moduleName + ".autoAssistEvent");
        tiltEvent = new TrcEvent(moduleName + ".tiltEvent");
        distanceEvent = new TrcEvent(moduleName + ".distanceEvent");
        intakeEvent = new TrcEvent(moduleName + ".intakeEvent");
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START);
    }   //CmdAutoStartPos1Or3

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
                case START:
                    // Read autoChoices.
                    // alliance = FrcAuto.autoChoices.getAlliance();
                    // startPos = FrcAuto.autoChoices.getStartPos();   // 0, 1, or 2.
                    // scorePreload = FrcAuto.autoChoices.getScorePreload();
                    // scoreLevel = FrcAuto.autoChoices.getScoreLevel();
                    // doAutoBalance = FrcAuto.autoChoices.getDoAutoBalance();
                    // scoreSecondPiece = FrcAuto.autoChoices.getScoreSecondPiece();

                    // Set robot's absolute field position according to the start position in autoChoices.
                    robot.robotDrive.setFieldPosition(null, false);

                    if((alliance == Alliance.Blue && startPos == 0) || (alliance == Alliance.Red && startPos == 2))
                    {
                        xOffset = -RobotParams.EXIT_COMMUNITY_X_OFFSET_MAGNITUDE;
                    }
                    else if ((alliance == Alliance.Blue && startPos == 2) || (alliance == Alliance.Red && startPos == 0))
                    {
                        xOffset = RobotParams.EXIT_COMMUNITY_X_OFFSET_MAGNITUDE;
                    }

                    if (scorePreload && scoreLevel == 0)
                    {
                        // Deploying & Spinning intake before backing up to reduce chance of cube bouncing out
                        robot.intake.extend();
                        robot.intake.setPower(0.2, -0.4, -0.4, 0.2, intakeEvent);
                        sm.waitForSingleEvent(intakeEvent, State.BACK_UP);
                    }
                    else
                    {
                        sm.setState(State.BACK_UP);
                    }                    
                    break;

                case BACK_UP:
                    // Back up a little so autoScore can raise the arm without hitting the shelf, and signal event when done.
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.5);
                    robot.robotDrive.purePursuitDrive.start(
                        driveEvent, 1.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(0.0, -24.0, 0.0));

                    if (!scorePreload || scoreLevel == 0)
                    {
                        // If we don't need to score or have already scored, check if we want to untuck before
                        // checking if we want to balance or not.
                        if (untuck)
                        {
                            nextState = State.UNTUCK_ARM;
                        }
                        else
                        {
                            robot.intake.retract();
                            nextState = scoreSecondPiece? State.GET_SECOND: (doAutoBalance? State.DRIVE_TO_BALANCE: State.DONE);
                        }
                    }
                    else
                    {
                        // We are scoring on a higher level, requiring the arm to be untucked
                        nextState = State.UNTUCK_ARM;
                    }
                    sm.waitForSingleEvent(driveEvent, nextState);
                    break;

                case UNTUCK_ARM:
                    robot.intake.extend();
                    robot.elevator.setAutoStartOffset(RobotParams.ELEVATOR_AUTOSTART_OFFSET);
                    robot.elevatorPidActuator.setPosition(
                        RobotParams.ELEVATOR_SAFE_HEIGHT, true, 1.0, elevatorEvent, 0.5);
                    robot.armPidActuator.setPosition(
                        null, 0.7, RobotParams.ARM_TRAVEL_POSITION, true, RobotParams.ARM_MAX_POWER,
                        null, 0.0);
                    robot.intake.retract(0.8);
                    if (scorePreload && scoreLevel > 0)
                    {
                        nextState = State.SCORE;
                    }
                    else
                    {
                        nextState = scoreSecondPiece? State.GET_SECOND: (doAutoBalance? State.DRIVE_TO_BALANCE: State.DONE);
                    }
                    sm.waitForSingleEvent(elevatorEvent, nextState);
                    break;

                case SCORE:
                    if (piecesScored == 0)
                    {
                        // Call autoScore to score the object.
                        robot.autoScoreTask.autoAssistScoreObject(
                            ObjectType.CUBE, scoreLevel, ScoreLocation.MIDDLE, false, autoAssistEvent);
                        sm.waitForSingleEvent(autoAssistEvent, (scoreSecondPiece? State.GET_SECOND: (doAutoBalance? State.DRIVE_TO_BALANCE: State.DONE)));
                    }
                    else
                    {
                        // Call autoScore to score the object.
                        robot.autoScoreTask.autoAssistScoreObject(
                            ObjectType.CONE, scoreLevel, ScoreLocation.RIGHT, false, autoAssistEvent);
                        sm.waitForSingleEvent(autoAssistEvent, State.DRIVE_TO_BALANCE);
                    }
                    piecesScored++;
                    break;

                case GET_SECOND:
                    robot.robotDrive.purePursuitDrive.start(
                        null, 5.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(xOffset, 144.0, 180.0));
                    timer.set(3.5, null, new TrcEvent.Callback() {
                        public void notify(Object context)
                        {
                            robot.autoPickupTask.autoAssistPickup(ObjectType.CONE, true, doAutoBalance, autoAssistEvent);
                        }
                    }, null);
                    sm.waitForSingleEvent(autoAssistEvent, State.DONE);//DRIVE_TO_SCORE);
                    break;
                
                case DRIVE_TO_SCORE:
                    //TODO: Check match time
                    robot.robotDrive.purePursuitDrive.start(
                        driveEvent, 1.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(-xOffset, 144.0, 0.0));
                    sm.waitForSingleEvent(driveEvent, State.SCORE);
                    break;

                case DRIVE_TO_BALANCE:
                    if (robot.elevator.getPosition() >= RobotParams.ELEVATOR_SAFE_HEIGHT)
                    {
                        // Lower elevator before moving (AutoScore usually does this but we aren't scoring here)
                        robot.elevatorPidActuator.setPosition(
                            null, 1.0, RobotParams.ELEVATOR_MIN_POS, true, 1.0, elevatorEvent, 0.5);
                    }
                    robot.robotDrive.purePursuitDrive.start(
                        driveEvent, 1, robot.robotDrive.driveBase.getFieldPosition(), true, 
                        new TrcPose2D(72, 0.0, 90),
                        new TrcPose2D(0.0, 0.0, 180));
                    sm.waitForSingleEvent(driveEvent, State.BALANCE);
                    break;

                case BALANCE:
                    // We're now next to the station inside of community, so we can do autobalance!
                    robot.robotDrive.driveBase.stop();
                    robot.robotDrive.disableDistanceTrigger();
                    robot.robotDrive.disableTiltTrigger();
                    if (doAutoBalance)
                    {
                        robot.autoBalanceTask.autoAssistBalance(
                            piecesScored == 2? BalanceInitSide.INSIDE: BalanceInitSide.OUTSIDE, autoAssistEvent);
                        sm.waitForSingleEvent(autoAssistEvent, State.DONE);
                    }
                    else
                    {
                        sm.setState(State.DONE);
                    }
                    break;
                    

                // case SCORE_GAME_PIECE:
                //     //  Preconditions:
                //     //      Robot is at position that it can see AprilTag (if using vision) and far enough to deploy elevator and
                //     //      arm without hitting field elements.
                //     if (piecesScored == 0)
                //     {
                //         // TODO (Code Review): Since we are StartPos 1 or 3, do we really need delay?
                //         //we havent scored anything, so we are scoring the preload and going to startdelay
                //         nextState = State.START_DELAY;
                //         robot.autoScoreTask.autoAssistScoreObject(
                //             ObjectType.CUBE, 2, ScoreLocation.MIDDLE, false,  event);
                //         sm.waitForSingleEvent(event, nextState);

                //     }
                //     else
                //     {
                //         //we've already scored the preload, score the second piece on the low level (spit intake, set 1 second timer)
                //         nextState = doAutoBalance? State.GO_TO_CHARGING_STATION: State.DONE;
                //         robot.intake.setPower(moduleName, 0, RobotParams.INTAKE_CUBE_PICKUP_POWER, RobotParams.INTAKE_CUBE_PICKUP_POWER, 0.5);
                //         TrcTimer timer = new TrcTimer(moduleName); 
                //         timer.set(1, event);
                //         sm.waitForSingleEvent(event, nextState);

                //     }
                //     piecesScored++;
                //     break;

                // case START_DELAY:
                //     if (scoreSecondPiece || doAutoBalance)
                //     {
                //         nextState = State.GO_TO_GAME_PIECE;
                //     }
                //     else
                //     {
                //         nextState = State.DONE;
                //     }
                //     double startDelay = FrcAuto.autoChoices.getStartDelay();
                //     if (startDelay == 0.0)
                //     {
                //         sm.setState(nextState);
                //     }
                //     else
                //     {
                //         sm.waitForSingleEvent(event, nextState);
                //         timer.set(startDelay, event);
                //     }
                //     break;

                // case GO_TO_GAME_PIECE:
                //     // Drives to a 3ft behind the game piece we want to pick up,
                //     //  determining the location by our alliance and startpos
                //     if (startPos == 0 && alliance == Alliance.Blue || startPos == 2 && alliance == Alliance.Red)
                //     {
                //         // We are going for the game piece on the guardrail side.
                //         robot.robotDrive.purePursuitDrive.start(
                //             event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                //             robot.robotDrive.adjustPosByAlliance(
                //                 alliance,
                //                 new TrcPose2D(RobotParams.CENTER_BETWEEN_CHARGING_STATION_AND_FIELD_EDGE_X,
                //                               RobotParams.CHARGING_STATION_CENTER_BLUE_Y, 180.0)),
                //             robot.robotDrive.adjustPosByAlliance(
                //                 alliance,
                //                 new TrcPose2D(RobotParams.GAME_PIECE_1_X, RobotParams.GAME_PIECE_BLUE_Y - 36.0, 0.0)));
                //     }
                //     else
                //     {
                //         // We are going for the game piece on the substation side.
                //         robot.robotDrive.purePursuitDrive.start(
                //             event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                //             new TrcPose2D(0, 0, 0.0),
                //             new TrcPose2D(0, 0, 0));
                //     }
                //      /* we wont have time to go for a third piece, keeping it just in case
                //     else if (piecesScored == 2)
                //     {
                //         // We are going for the third game piece.
                //         // drives to the location of the second right most piece
                //         if (FrcAuto.autoChoices.getAlliance() == Alliance.Blue)
                //         {
                //             robot.robotDrive.purePursuitDrive.start(
                //                 event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                //                 new TrcPose2D(RobotParams.CENTER_BETWEEN_CHARGING_STATION_AND_FIELD_EDGE_X, 85.0, 0.0),
                //                 new TrcPose2D(RobotParams.CENTER_BETWEEN_CHARGING_STATION_AND_FIELD_EDGE_X, 220.0, 0.0),
                //                 new TrcPose2D(RobotParams.GAME_PIECE_2_X, RobotParams.GAME_PIECE_BLUE_Y - 36, 0.0));
                //         }
                //         else
                //         {
                //             robot.robotDrive.purePursuitDrive.start(
                //                 event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                //                 new TrcPose2D(RobotParams.CENTER_BETWEEN_CHARGING_STATION_AND_FIELD_EDGE_X, 564.0, 180.0),
                //                 new TrcPose2D(RobotParams.CENTER_BETWEEN_CHARGING_STATION_AND_FIELD_EDGE_X, 429.0, 180.0),
                //                 new TrcPose2D(RobotParams.GAME_PIECE_2_X, RobotParams.GAME_PIECE_RED_Y + 36, 180.0));
                //         }
                //     } */
                //     sm.waitForSingleEvent(event, State.PICKUP_GAME_PIECE);
                //     break;
                    
                // case PICKUP_GAME_PIECE:
                //     // Drives forward while running intake until it picks up a game piece, the precondition being that
                //     // we already at the correct location
                //     if (scoreSecondPiece) {
                //         nextState = State.GO_TO_SCORE_POSITION;
                //     }
                //     else {
                //         nextState = State.GO_TO_CHARGING_STATION;
                //     }
                //     // pick up cube with approach only(only sucks it into the weedwhacker)
                //     robot.autoPickupTask.autoAssistPickupApproachOnly(
                //         //piecesScored == 1? ObjectType.CUBE: Right now, we are assuming that we can pick up a cone reliably
                //         ObjectType.CUBE, useVision, event);
                //     //if (piecesScored == 1) {
                //     //    loadedObjType = ObjectType.CUBE;
                //     //}
                //     //else {
                //     // loadedObjType = ObjectType.CONE;
                //     sm.waitForSingleEvent(event, nextState);
                //     break;

                // //drive in front of the rightmost/leftmost pole to prepare for scoring cube on low 
                // case GO_TO_SCORE_POSITION:
                //     //we will be scoring low next, so spin intake to make sure cube doesn't fall out like teleop 
                //     robot.intake.setPower(moduleName, 0, RobotParams.INTAKE_CUBE_PICKUP_POWER, RobotParams.INTAKE_CUBE_PICKUP_POWER, 0.0);
                //     //drives in front of the rightmost pole to score cube on low
                //     //startPos 0 and startPos 2 should be the equivalent of 1 and 3 on shuffleboard
                //     if (startPos == 0 && alliance == Alliance.Blue || startPos == 2 && alliance == Alliance.Red)
                //     {
                //         robot.robotDrive.purePursuitDrive.start(
                //             event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                //             //intermediate point so that we turn before entering the community
                //             robot.robotDrive.adjustPosByAlliance(
                //                 alliance, 
                //                 new TrcPose2D(RobotParams.CENTER_BETWEEN_CHARGING_STATION_AND_FIELD_EDGE_X, 220.0, 180.0)), 
                //             //right in front of the rightmost pole, may need to tune 
                //             robot.robotDrive.adjustPosByAlliance(
                //                 alliance,
                //                 new TrcPose2D(-14.2, 75.0, 180.0)));
                //     }
                //     //drive in front of the leftmost pole(startpos 2 for blue, startpos 0 for red)
                //     else
                //     {
                //         robot.robotDrive.purePursuitDrive.start(
                //             event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                //             //intermediate point so that we turn before entering the community
                //             robot.robotDrive.adjustPosByAlliance(
                //                 alliance, 
                //                 new TrcPose2D(-186, 220.0, 180.0)), 
                //             //right in front of the leftmost pole, may need to tune 
                //             robot.robotDrive.adjustPosByAlliance(
                //                 alliance,
                //                 new TrcPose2D(-200, 75.0, 180.0)));
                //     }
                //     /* again we are not going to have time to get a 3rd piece, keeping just in case
                //     else if (piecesScored == 2)
                //     {
                //         // drives to the score precondition position from the second right most game piece
                //         if (FrcAuto.autoChoices.getAlliance() == Alliance.Blue)
                //         {
                //             robot.robotDrive.purePursuitDrive.start(
                //                 event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                //                 new TrcPose2D(RobotParams.CENTER_BETWEEN_CHARGING_STATION_AND_FIELD_EDGE_X, 220.0, 180.0),
                //                 new TrcPose2D(RobotParams.CENTER_BETWEEN_CHARGING_STATION_AND_FIELD_EDGE_X, 85.0, 180.0));
                //         }
                //         else
                //         {
                //             robot.robotDrive.purePursuitDrive.start(
                //                 event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                //                 new TrcPose2D(RobotParams.CENTER_BETWEEN_CHARGING_STATION_AND_FIELD_EDGE_X, 429, 0.0),
                //                 new TrcPose2D(RobotParams.CENTER_BETWEEN_CHARGING_STATION_AND_FIELD_EDGE_X, 564.0, 0.0));
                //         }
                //     }*/
                //     sm.waitForSingleEvent(event, State.SCORE_GAME_PIECE);
                //     break;

                // case GO_TO_CHARGING_STATION:
                //     // Drives behind the charging station platform from the scoring position
                //     if (scoreSecondPiece)
                //     {
                //         //since we just scored the second piece, we are inside the community so we get onto the charging station from the inside
                //         robot.robotDrive.purePursuitDrive.start(
                //             event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                //             robot.robotDrive.adjustPosByAlliance(
                //                 alliance,
                //                 new TrcPose2D(
                //                     RobotParams.CHARGING_STATION_CENTER_X,
                //                     RobotParams.CHARGING_STATION_CENTER_BLUE_Y - (RobotParams.CHARGING_STATION_DEPTH + 15),
                //                     270)));
                //     }
                //     else
                //     {
                //         //since we aren't scoring the second piece, we're in the center of the field and will get onto the charging station from the outside
                //         robot.robotDrive.purePursuitDrive.start(
                //             event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                //             robot.robotDrive.adjustPosByAlliance(
                //                 alliance,
                //                 new TrcPose2D(
                //                     RobotParams.CHARGING_STATION_CENTER_X,
                //                     RobotParams.CHARGING_STATION_CENTER_BLUE_Y + (RobotParams.CHARGING_STATION_DEPTH + 15),
                //                     90)));
                //     }
                //     sm.waitForSingleEvent(event, State.GET_ON_CHARGING_STATION);
                //     break;

                // case GET_ON_CHARGING_STATION: //we crab onto the charging station in relative
                //     // Auto Balance requires that the robot is on the floor next to the charging station, not already on it
                //     // robot.robotDrive.purePursuitDrive.start(
                //     //     event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                //     //     new TrcPose2D(RobotParams.CHARGING_STATION_DEPTH + 15, 0, 0));
                //     break;

                case EXIT_COMMUNITY:
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.3);
                    robot.robotDrive.purePursuitDrive.start(
                            driveEvent, 4.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                            new TrcPose2D(xOffset, -156.0, 0.0));
                    sm.waitForSingleEvent(driveEvent, State.DONE);
                    break;

                case DONE:
                default:
                    // We are done.
                    cancel();
                    break;
            }

            robot.globalTracer.traceStateInfo(
                sm.toString(), state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                robot.robotDrive.purePursuitDrive, null);
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdAutoStartPos1Or3
