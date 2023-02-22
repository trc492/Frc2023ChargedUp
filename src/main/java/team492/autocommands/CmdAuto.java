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
import team492.autotasks.TaskAutoBalance;
import team492.autotasks.TaskAutoPickup;
import team492.autotasks.TaskScoreObject;
import team492.autotasks.TaskScoreObject.ObjectType;

public class CmdAuto implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAuto";

    private enum State
    {
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
    private final TrcEvent timerEvent;
    private final TrcEvent grabberEvent;
    private final TrcEvent driveEvent;
    private final TrcEvent intakeEvent;
    private final TrcStateMachine<State> sm;
    private final TaskAutoPickup autoPickupTask;
    private final TaskScoreObject autoScoreTask;
    private final TaskAutoBalance autoBalanceTask;

    private int piecesScored = 0;
    private boolean autoBalance = false;//if true, we auto-balance, if false, we try to score a third piece 
    //TODO: not sure if we should connect with auto choices or pass in as a parameter

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     */
    public CmdAuto(Robot robot)
    {
        robot.globalTracer.traceInfo(moduleName, ">>> robot=%s, choices=%s", robot, FrcAuto.autoChoices);

        this.robot = robot;
        timer = new TrcTimer(moduleName + ".timer");
        timerEvent = new TrcEvent(moduleName + ".timerEvent");
        grabberEvent = new TrcEvent(moduleName + ".grabberEvent");
        driveEvent = new TrcEvent(moduleName + ".driveEvent");
        intakeEvent = new TrcEvent(moduleName + ".intakeEvent");
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START_DELAY);
        autoPickupTask = new TaskAutoPickup(moduleName, robot, robot.globalTracer);
        autoScoreTask = new TaskScoreObject(moduleName, robot, robot.globalTracer);
        autoBalanceTask = new TaskAutoBalance(moduleName, robot, robot.globalTracer);
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
        // autoPickupTask.autoAssistCancel();
        autoScoreTask.autoAssistCancel();
        autoBalanceTask.autoAssistCancel();
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
                case SCORE_GAME_PIECE:
                    // Scores a game piece, the precondition being that it is already in the scoring position, with
                    // a game piece in the robot, broadacasting grabberEvent when complete.
                    // autoScoreTask.autoAssistScoreObject(...);
                    autoScoreTask.autoAssistScoreObject(ObjectType.CUBE, true, 2, grabberEvent);
                    piecesScored++;
                    if (piecesScored == 1) {
                        sm.waitForSingleEvent(grabberEvent, State.START_DELAY);
                    }
                    else if (piecesScored == 2 && !autoBalance) {
                        sm.waitForSingleEvent(grabberEvent, State.GO_TO_GAME_PIECE);
                    }
                    else {
                        sm.waitForSingleEvent(grabberEvent, State.GO_TO_CHARGING_STATION);
                    }
                    break;

                case START_DELAY:
                    double startDelay = FrcAuto.autoChoices.getStartDelay();
                    if (startDelay == 0.0)
                    {
                        sm.setState(State.GO_TO_GAME_PIECE);
                    }
                    else
                    {
                        sm.waitForSingleEvent(timerEvent, State.GO_TO_GAME_PIECE);
                        timer.set(startDelay, timerEvent);
                    }
                    break;

                case GO_TO_GAME_PIECE:
                    // Drives to a few feet (3ft from the center of the robot to the ball) behind the game piece we
                    // want to pick up, determining the location by how many pieces we have already scored
                    if (piecesScored == 1) {
                        //drives to 3ft behind the right most piece from the init scoring pos
                        if (FrcAuto.autoChoices.getAlliance() == Alliance.Blue) {
                            robot.robotDrive.purePursuitDrive.start(
                                driveEvent, 2.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                new TrcPose2D(-33.0, 85.0, 0.0),
                                new TrcPose2D(-33.0, 238.0, 0.0));
                        }
                        else {
                            robot.robotDrive.purePursuitDrive.start(
                                driveEvent, 2.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                new TrcPose2D(-33.0, 564.0, 180.0),
                                new TrcPose2D(-33.0, 404.0, 180.0));
                        }
                    }
                    else if (piecesScored == 2) {
                        //drives to the location of the second right most piece from second scoring pos (shelf closest to scoring table)
                        if (FrcAuto.autoChoices.getAlliance() == Alliance.Blue) {
                            robot.robotDrive.purePursuitDrive.start(
                                driveEvent, 2.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                new TrcPose2D(-33.0, 85.0, 0.0),
                                new TrcPose2D(-33.0, 220.0, 0.0),
                                new TrcPose2D(-82.0, 238.0, 0.0));

                        }
                        else {
                            robot.robotDrive.purePursuitDrive.start(
                                driveEvent, 2.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                new TrcPose2D(-33.0, 564.0, 180.0),
                                new TrcPose2D(-33.0, 429.0, 180.0),
                                new TrcPose2D(-82.0, 404.0, 180.0));
                        }
                    }
                    sm.waitForSingleEvent(driveEvent, State.PICKUP_GAME_PIECE);
                    break;
                    
                case PICKUP_GAME_PIECE:
                    // Drives forward while running intake until it picks up a game piece, the precondition being that
                    // we already at the correct location
                    autoPickupTask.start(intakeEvent, 2.0);
                    //we dont need the folowing lines, keeping it just in case
                    // robot.robotDrive.purePursuitDrive.start(
                    //     null, 2.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                    //     new TrcPose2D(0, 50, 0));
                    sm.waitForSingleEvent(intakeEvent, State.GO_TO_SCORE_POSITION);
                    break;

                case GO_TO_SCORE_POSITION:
                    // Drives to the scoring position, determining the location based on how many pieces we have
                    // already scored
                    if (piecesScored == 1) {
                        //drives to the right most shelf from the right most game piece
                        if (FrcAuto.autoChoices.getAlliance() == Alliance.Blue) {
                            robot.robotDrive.purePursuitDrive.start(
                                driveEvent, 2.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                new TrcPose2D(-33.0, 85.0, 180.0),
                                new TrcPose2D(-40.0, 65.0, 180.0));
                        }
                        else {
                            robot.robotDrive.purePursuitDrive.start(
                                driveEvent, 2.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                new TrcPose2D(-33.0, 564.0, 0.0),
                                new TrcPose2D(-40.0, 584.0, 0.0));
                        }
                    }
                    else if (piecesScored == 2) {
                        // drives to the right most shelf from the second right most game piece
                        if (FrcAuto.autoChoices.getAlliance() == Alliance.Blue) {
                            robot.robotDrive.purePursuitDrive.start(
                                driveEvent, 2.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                new TrcPose2D(-33.0, 220.0, 180.0),
                                new TrcPose2D(-33.0, 85.0, 180.0),
                                new TrcPose2D(-40.0, 65.0, 180.0));
                        }
                        else {
                            robot.robotDrive.purePursuitDrive.start(
                                driveEvent, 2.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                new TrcPose2D(-33.0, 429, 0.0),
                                new TrcPose2D(-33.0, 564.0, 0.0),
                                new TrcPose2D(-40.0, 580.0, 0.0));
                        }
                    }
                    sm.waitForSingleEvent(driveEvent, State.SCORE_GAME_PIECE);
                    break;

                case GO_TO_CHARGING_STATION:
                    // Drives to just behind and then onto the charging station platform from the scoring position
                    if (FrcAuto.autoChoices.getAlliance() == Alliance.Blue) {
                        robot.robotDrive.purePursuitDrive.start(
                            driveEvent, 2.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            new TrcPose2D(-107.0, 85.0, 0),
                            new TrcPose2D(-107.0, 145.0, 0));
                    }
                    else {
                        robot.robotDrive.purePursuitDrive.start(
                            driveEvent, 2.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            new TrcPose2D(-107.0, 550.0, 180),
                            new TrcPose2D(-107.0, 490.0, 180));
                    }
                    sm.waitForSingleEvent(driveEvent, State.AUTO_BALANCE);
                    break;

                case AUTO_BALANCE:
                    autoBalanceTask.autoAssistBalance(driveEvent);
                    sm.waitForSingleEvent(driveEvent, State.DONE);
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
