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
    private final TrcEvent intakeEvent;
    private final TrcStateMachine<State> sm;

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
                    robot.elevator.setAutoStartOffset(RobotParams.ELEVATOR_AUTOSTART_OFFSET);

                    if (scorePreload && scoreLevel == 0)
                    {
                        // Deploying & spinning intake to score the preloaded cube to ground level.
                        robot.intake.extend();
                        robot.intake.setPower(0.2, -0.4, -0.4, 0.5, intakeEvent);
                        sm.waitForSingleEvent(intakeEvent, State.BACK_UP);
                        piecesScored++;
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
                        driveEvent, 0.8, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(0.0, -24.0, 0.0));
                    sm.waitForSingleEvent(driveEvent, State.UNTUCK_ARM);
                    // if (!scorePreload || scoreLevel == 0)
                    // {
                    //     // TODO (Code Review): Why don't we always untuck? Is there a scenario that we don't want to?
                    //     // If we don't need to score or have already scored, check if we want to untuck before
                    //     // checking if we want to balance or not.
                    //     if (untuck)
                    //     {
                    //         nextState = State.UNTUCK_ARM;
                    //     }
                    //     else
                    //     {
                    //         robot.intake.retract(0.5);
                    //         // TODO (Code Review): If your arm is still tucked in, how are you getting the second piece???
                    //         nextState = scoreSecondPiece? State.GET_SECOND: (doAutoBalance? State.DRIVE_TO_BALANCE: State.DONE);
                    //     }
                    // }
                    // else
                    // {
                    //     // We are scoring on a higher level, requiring the arm to be untucked.
                    //     nextState = State.UNTUCK_ARM;
                    // }
                    // sm.waitForSingleEvent(driveEvent, nextState);
                    break;

                case UNTUCK_ARM:
                    robot.intake.extend();
                    robot.elevatorPidActuator.setPosition(
                        RobotParams.ELEVATOR_SAFE_HEIGHT, true, 1.0, elevatorEvent, 0.5);
                    robot.armPidActuator.setPosition(
                        null, 0.7, RobotParams.ARM_TRAVEL_POSITION, true, RobotParams.ARM_MAX_POWER,
                        null, 0.0);
                    robot.intake.retract(0.9);
                    if (scorePreload && scoreLevel > 0)
                    {
                        nextState = State.SCORE;
                    }
                    else
                    {
                        nextState = State.EXIT_COMMUNITY;
                        // nextState = scoreSecondPiece? State.GET_SECOND: (doAutoBalance? State.DRIVE_TO_BALANCE: State.DONE);
                    }
                    sm.waitForSingleEvent(elevatorEvent, nextState);
                    break;

                case SCORE:
                    robot.autoScoreTask.autoAssistScoreObject(
                        ObjectType.CUBE, scoreLevel, ScoreLocation.MIDDLE, false, autoAssistEvent);
                    sm.waitForSingleEvent(autoAssistEvent, State.EXIT_COMMUNITY);
                    // sm.waitForSingleEvent(autoAssistEvent, (doAutoBalance? State.TURN: State.DONE));
                    // if (piecesScored == 0)
                    // {
                    //     // AutoScore the preloaded object high.
                    //     robot.autoScoreTask.autoAssistScoreObject(
                    //         ObjectType.CUBE, scoreLevel, ScoreLocation.MIDDLE, false, autoAssistEvent);
                    //     sm.waitForSingleEvent(
                    //         autoAssistEvent,
                    //         (scoreSecondPiece? State.GET_SECOND: (doAutoBalance? State.DRIVE_TO_BALANCE: State.DONE)));
                    // }
                    // else
                    // {
                    //     // AutoScore the second piece.
                    //     // TODO (Code Review): where do you want to score the second piece? What level? The space could
                    //     // have been occupied by the preload!
                    //     robot.autoScoreTask.autoAssistScoreObject(
                    //         ObjectType.CUBE, scoreLevel, ScoreLocation.MIDDLE, false, autoAssistEvent);
                    //     sm.waitForSingleEvent(autoAssistEvent, doAutoBalance? State.DRIVE_TO_BALANCE: State.DONE);
                    // }
                    piecesScored++;
                    break;

                case EXIT_COMMUNITY:
                    if((alliance == Alliance.Blue && startPos == 0) || (alliance == Alliance.Red && startPos == 2))
                    {
                        xOffset = -RobotParams.EXIT_COMMUNITY_X_OFFSET_MAGNITUDE;
                    }
                    else if ((alliance == Alliance.Blue && startPos == 2) || (alliance == Alliance.Red && startPos == 0))
                    {
                        xOffset = RobotParams.EXIT_COMMUNITY_X_OFFSET_MAGNITUDE;
                    }

                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.3);
                    robot.robotDrive.purePursuitDrive.start(
                            driveEvent, 4.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                            new TrcPose2D(xOffset, -156.0, 0.0));
                    sm.waitForSingleEvent(driveEvent, doAutoBalance? State.DRIVE_TO_BALANCE: State.DONE);
                    break;

                // case GET_SECOND:
                //     robot.robotDrive.purePursuitDrive.setMsgTracer(robot.globalTracer, true, true);
                //     robot.robotDrive.purePursuitDrive.start(
                //         driveEvent, 5.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                //         new TrcPose2D(xOffset, 144.0, 180.0));
                //     sm.waitForSingleEvent(driveEvent, State.DRIVE_TO_SCORE);
                //     // timer.set(3.5, null, new TrcEvent.Callback() {
                //     //     public void notify(Object context)
                //     //     {
                //     //         robot.autoPickupTask.autoAssistPickup(ObjectType.CONE, true, doAutoBalance, autoAssistEvent);
                //     //     }
                //     // }, null);
                //     // sm.waitForSingleEvent(autoAssistEvent, State.DONE);//DRIVE_TO_SCORE);
                //     break;
                
                // case DRIVE_TO_SCORE:
                //     //TODO: Check match time
                //     robot.robotDrive.purePursuitDrive.start(
                //         driveEvent, 1.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                //         new TrcPose2D(-xOffset, 144.0, 0.0));
                //     sm.waitForSingleEvent(driveEvent, State.SCORE);
                //     break;

                case DRIVE_TO_BALANCE:
                    if (robot.elevator.getPosition() >= RobotParams.ELEVATOR_SAFE_HEIGHT)
                    {
                        // Lower elevator before moving (AutoScore usually does this but we aren't scoring here)
                        robot.elevatorPidActuator.setPosition(
                            null, 1.0, RobotParams.ELEVATOR_MIN_POS, true, 1.0, elevatorEvent, 0.5);
                    }

                    TrcPose2D balancePose;
                    if (alliance == Alliance.Blue)
                    {
                        balancePose = new TrcPose2D(
                            RobotParams.STARTPOS_2_X, RobotParams.STARTPOS_BLUE_Y + 144.0, -90.0);
                    }
                    else
                    {
                        balancePose = new TrcPose2D(
                            RobotParams.STARTPOS_2_X, RobotParams.STARTPOS_RED_Y - 144.0, 90.0);
                    }

                    robot.robotDrive.purePursuitDrive.start(
                        driveEvent, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                        balancePose);
                    sm.waitForSingleEvent(driveEvent, State.BALANCE);
                    break;

                case BALANCE:
                    // We're now next to the station outside of community, so we can do autobalance!
                    robot.autoBalanceTask.autoAssistBalance(BalanceInitSide.OUTSIDE, autoAssistEvent);
                    sm.waitForSingleEvent(autoAssistEvent, State.DONE);
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
