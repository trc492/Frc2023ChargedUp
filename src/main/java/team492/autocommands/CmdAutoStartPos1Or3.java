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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import team492.FrcAuto;
import team492.Robot;
import team492.RobotParams;
import team492.FrcAuto.AutoStartPos;
import team492.FrcAuto.BalanceInitSide;
import team492.FrcAuto.ObjectType;
import team492.FrcAuto.ScoreLocation;

public class CmdAutoStartPos1Or3 implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoStartPos1Or3";

    private enum State
    {
        START,
        EXIT_COMMUNITY,
        // EXIT_COMMUNITY2,
        PICKUP_SECOND_CUBE,
        PREP_SCORE_SECOND_CUBE,
        SCORE_SECOND_CUBE,
        DRIVE_TO_BALANCE,
        BALANCE,
        DONE
    }   //enum State


    private final Robot robot;
    private final TrcEvent driveEvent;
    private final TrcEvent autoAssistEvent;
    private final TrcStateMachine<State> sm;

    private Alliance alliance = Alliance.Blue;
    private AutoStartPos startPos = AutoStartPos.BARRIER;
    private boolean scorePreload = true;
    private ObjectType preloadType = ObjectType.CUBE;
    private int scoreLevel = 0;
    private ScoreLocation scoreLocation = ScoreLocation.RIGHT;
    private boolean doAutoBalance = false;
    private boolean scoreSecondPiece = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     */
    public CmdAutoStartPos1Or3(Robot robot)
    {
        this.robot = robot;
        driveEvent = new TrcEvent(moduleName + ".driveEvent");
        autoAssistEvent = new TrcEvent(moduleName + ".autoAssistEvent");
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
            robot.dashboard.displayPrintf(8, "State: %s", state);
            switch (state)
            {
                case START:
                    // Read autoChoices.
                    alliance = FrcAuto.autoChoices.getAlliance();
                    startPos = FrcAuto.autoChoices.getStartPos();
                    scorePreload = FrcAuto.autoChoices.getScorePreload();
                    preloadType = FrcAuto.autoChoices.getPreloadedObjType();
                    scoreLevel = FrcAuto.autoChoices.getScoreLevel();
                    scoreLocation = FrcAuto.autoChoices.getScoreLocation();
                    doAutoBalance = FrcAuto.autoChoices.getDoAutoBalance();
                    scoreSecondPiece = FrcAuto.autoChoices.getScoreSecondPiece();
                    // Set robot's absolute field position according to the start position in autoChoices.
                    robot.robotDrive.setFieldPosition(null, false);

                    robot.elevatorPidActuator.zeroCalibrate();
                    robot.wristPidActuator.setPosition(RobotParams.WRIST_MIN_POS, true);

                    if (scorePreload)
                    {
                        robot.autoScoreTask.autoAssistScoreObject(
                            preloadType, scoreLevel, scoreLocation, false, false, autoAssistEvent);
                        sm.waitForSingleEvent(autoAssistEvent, doAutoBalance? State.EXIT_COMMUNITY: State.DONE);
                    }
                    else
                    {
                        sm.setState(State.EXIT_COMMUNITY);
                        // sm.setState(doAutoBalance? State.EXIT_COMMUNITY: State.DONE);
                    }
                    break;

                case EXIT_COMMUNITY:
                    double xDelta = startPos == AutoStartPos.BARRIER? -8.0: 8.0;
                    double yDelta = alliance == Alliance.Blue? 180.0: -180.0;
                    TrcPose2D robotPose = robot.robotDrive.driveBase.getFieldPosition();
                    robot.robotDrive.pidDrive.setMsgTracer(robot.globalTracer, true, true);
                    robot.robotDrive.pidDrive.setAbsoluteTarget(robotPose.x + xDelta, robotPose.y + yDelta, robotPose.angle, driveEvent);
                    sm.waitForSingleEvent(driveEvent, State.DONE);
                    // robot.robotDrive.enableDistanceTrigger(6.0, driveEvent);
                    // double xPower = alliance == Alliance.Blue && startPos == AutoStartPos.BARRIER ||
                    //                 alliance == Alliance.Red && startPos == AutoStartPos.FIELDRAIL? -0.2: 0.2;
                    // robot.robotDrive.driveBase.holonomicDrive(
                    //     null, xPower, 0.0, 0.0, robot.robotDrive.driveBase.getHeading());
                    // sm.waitForSingleEvent(driveEvent, State.EXIT_COMMUNITY2);//PICKUP_SECOND_CUBE);
                    break; 

                // case EXIT_COMMUNITY2:
                //     robot.robotDrive.enableDistanceTrigger(160.0, driveEvent);
                //     double yPower = alliance == Alliance.Blue? 0.5: -0.5;
                //     robot.robotDrive.driveBase.holonomicDrive(
                //         null, 0, yPower, 0, robot.robotDrive.driveBase.getHeading());
                //     sm.waitForSingleEvent(driveEvent, State.DONE);
                //     break;

                case PICKUP_SECOND_CUBE:
                    robot.prepForCubeGroundPickup(null, 0.0, null);

                    sm.addEvent(autoAssistEvent);
                    robot.intake.autoAssistIntake(
                        0.0, RobotParams.INTAKE_PICKUP_POWER, RobotParams.INTAKE_CUBE_RETAIN_POWER, 0.75,
                        autoAssistEvent, 0.0);

                    sm.addEvent(driveEvent);
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.3);
                    if (startPos == AutoStartPos.FIELDRAIL)
                    {
                        // We are going for the game piece on the guardrail side.
                        robot.robotDrive.purePursuitDrive.start(
                            driveEvent, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.adjustPosByAlliance(
                                alliance,
                                new TrcPose2D(RobotParams.GAME_PIECE_1_X, RobotParams.GAME_PIECE_BLUE_Y, 0.0)));
                    }
                    else
                    {
                        // We are going for the game piece on the substation side.
                        robot.robotDrive.purePursuitDrive.start(
                            driveEvent, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.adjustPosByAlliance(
                                alliance,
                                new TrcPose2D(RobotParams.GAME_PIECE_4_X, RobotParams.GAME_PIECE_BLUE_Y, 0.0)));
                    }
                    sm.waitForEvents(State.PREP_SCORE_SECOND_CUBE, false);
                    break;

                case PREP_SCORE_SECOND_CUBE:
                    robot.robotDrive.purePursuitDrive.cancel();
                    robot.turtleMode(null, null);
                    sm.setState(scoreSecondPiece && robot.intake.hasObject()? State.SCORE_SECOND_CUBE:
                                doAutoBalance? State.DRIVE_TO_BALANCE: State.DONE);
                    break;

                case SCORE_SECOND_CUBE:
                    //TODO: To be written.
                    break;

                case DRIVE_TO_BALANCE:
                    double balancePosY = scoreSecondPiece? RobotParams.STARTPOS_BLUE_Y + 24.0: RobotParams.STARTPOS_BLUE_Y + 144.0;
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.5);
                    robot.robotDrive.purePursuitDrive.start(
                        driveEvent, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                        robot.robotDrive.adjustPosByAlliance(
                            alliance,
                            new TrcPose2D(RobotParams.CHARGING_STATION_CENTER_X, balancePosY, 90.0)));
                    sm.waitForSingleEvent(driveEvent, State.BALANCE);
                    break;

                case BALANCE:
                    // We're now next to the station outside of community, so we can do autobalance!
                    robot.autoBalanceTask.autoAssistBalance(
                        scoreSecondPiece? BalanceInitSide.INSIDE: BalanceInitSide.OUTSIDE, autoAssistEvent);
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
