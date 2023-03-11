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
import team492.FrcAuto;
import team492.Robot;
import team492.RobotParams;
import team492.FrcAuto.BalanceStrafeDir;
import team492.FrcAuto.ObjectType;
import team492.FrcAuto.ScoreLocation;
import team492.drivebases.RobotDrive;

public class CmdAutoStartPos2 implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoStartPos2";

    private enum State
    {
        START,
        SCORE_PRELOAD,
        TURN,
        START_TO_CLIMB,
        CLIMB,
        DESCEND,
        BALANCE,
        DONE

    }   //enum State


    private final Robot robot;
    private final TrcEvent event;
    private final TrcEvent tiltEvent;
    private final TrcStateMachine<State> sm;

    private int scoreLevel;
    private boolean scorePreload;
    private boolean doAutoBalance;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     */
    public CmdAutoStartPos2(Robot robot)
    {
        robot.globalTracer.traceInfo(moduleName, ">>> robot=%s, choices=%s", robot, FrcAuto.autoChoices);

        this.robot = robot;
        event = new TrcEvent(moduleName);
        tiltEvent = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.TURN);
    }   //CmdAutoStartPos2

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
                    scoreLevel = FrcAuto.autoChoices.getScoreLevel();
                    scorePreload = FrcAuto.autoChoices.getScorePreload();
                    doAutoBalance = FrcAuto.autoChoices.getDoAutoBalance();
                    // TODO (Code Review): setFieldPosition will no longer work since it was assuming robot touching GRID.
                    // May want to use vision to determine exact location, or you don't care about odometry in this auto.
                    robot.robotDrive.setFieldPosition(null, false);
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.3);
                    if (scorePreload)
                    {
                        robot.elevatorPidActuator.setPosition(
                            RobotParams.ELEVATOR_SAFE_HEIGHT, true, 1.0, event, 0.5);
                        robot.armPidActuator.setPosition(
                            moduleName, 0.2, RobotParams.ARM_TRAVEL_POSITION, true, RobotParams.ARM_MAX_POWER, null, 0.5);
                        sm.setState(State.SCORE_PRELOAD);
                    }
                    else
                    {
                        sm.setState(State.TURN);
                    }
                    break;

                case SCORE_PRELOAD:
                    robot.autoScoreTask.autoAssistScoreObject(
                        ObjectType.CUBE, scoreLevel, ScoreLocation.MIDDLE, false, event);
                    sm.waitForSingleEvent(event, State.TURN);
                    break;
                
                case TURN: // turn right to prepare to crab over the station
                    robot.robotDrive.purePursuitDrive.start(
                        //TODO: we might want to move over a bit to be more in the middle of the station
                        event, 1.5, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(0.0, 0.0, 90.0));
                    sm.waitForSingleEvent(event, State.START_TO_CLIMB);
                    break;

                case START_TO_CLIMB:
                    robot.robotDrive.enableTiltTrigger(tiltEvent);
                    robot.robotDrive.driveBase.holonomicDrive(0.3, 0.0, 0.0);
                    sm.waitForSingleEvent(tiltEvent, State.CLIMB, 5.0);
                    break;
                
                case CLIMB: // we're climbing up the station, going to the next state when we're level on the station
                    if (tiltEvent.isSignaled()) {
                        if (robot.robotDrive.inBalanceZone()) {
                            sm.setState(State.DESCEND);
                        }
                        else {
                            sm.waitForSingleEvent(tiltEvent, State.CLIMB);
                        }
                    }
                    else //we missed the platform because our angle hasn't changed
                    {
                        sm.setState(State.DONE);
                    }
                    break;

                case DESCEND: // we're descending down the other side of the station until we're flat on the ground, in which case we've left the community and can now go balance
                    if (tiltEvent.isSignaled()) {
                        if (robot.robotDrive.inBalanceZone()) {
                            robot.robotDrive.cancel();
                            sm.setState(State.BALANCE);
                        }
                        else {

                            sm.waitForSingleEvent(tiltEvent, State.DESCEND);
                        }
                    }
                    else
                    {
                        sm.setState(State.DONE);
                    }
                    break;

                case BALANCE: //we're now next to the station outside of community, so we can do autobalance!
                    if (doAutoBalance)
                    {
                        robot.autoBalanceTask.autoAssistBalance(BalanceStrafeDir.LEFT, event);
                        sm.waitForSingleEvent(event, State.DONE);
                    }
                    else
                    {
                        sm.setState(State.DONE);
                    }
                    break;

                default:
                case DONE:
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

}   //class CmdAutoStartPos2
