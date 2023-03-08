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

public class CmdAutoStartPos2 implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoStartPos2";

    private enum State
    {
        START,
        SCORE_PRELOAD,
        TURN,
        EXIT_COMMUNITY,
        CLIMB,
        LEVEL,
        DESCEND,
        EXIT,
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
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.25);
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
                
                case TURN:
                    robot.robotDrive.purePursuitDrive.start(
                        event, 1.5, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(0.0, 0.0, 90.0));
                    sm.waitForSingleEvent(event, State.CLIMB);
                    break;
                
                case EXIT_COMMUNITY:
                    // TODO (Code Review): Odometry doesn't work well going over charging station, should do the following:
                    // 1. arm tilt trigger with tiltEvent and do purePursuitDrive for a distance with event, goto CLIMB
                    //    when either event signaled.
                    // 2. if tilt event signaled, clear event and wait for tilt trigger again, goto LEVEL when either events
                    //    signaled.
                    // 3. if tilt event signaled, clear event and wait for tilt trigger again, goto DESCEND when either events
                    //    signaled.
                    // 4. if tilt event signaled, clear event and wait for tilt trigger again, goto EXIT when either events
                    //    signaled.
                    // 5. unarm tilt trigger, cancel purePursuit, call auto balance.
                    // 6. done.

                    // robot.robotDrive.purePursuitDrive.start(
                    //     event, 7.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                    //     new TrcPose2D(180.0, 0.0, 0.0));
                    // sm.waitForSingleEvent(event, doAutoBalance? State.BALANCE: State.BALANCE);
                    robot.robotDrive.setTiltTriggerEnabled(true, tiltEvent);
                    sm.addEvent(tiltEvent);
                    robot.robotDrive.purePursuitDrive.start(
                        event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(60.0, 0.0, 0.0));
                    sm.waitForEvents(State.CLIMB, false);
                    sm.addEvent(event);
                    break;
                case CLIMB:
                    if (tiltEvent.isSignaled()) {
                        event.clear();
                        sm.addEvent(tiltEvent);
                        robot.robotDrive.purePursuitDrive.start(
                            event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                            new TrcPose2D(60.0, 0.0, 0.0));
                        sm.addEvent(event);
                        sm.waitForEvents(State.LEVEL, false);
                    }
                    else {
                        sm.setState(State.DONE);
                    }
                    break;

                case LEVEL:
                    if (tiltEvent.isSignaled()) {
                        event.clear();
                        sm.addEvent(tiltEvent);
                        robot.robotDrive.purePursuitDrive.start(
                            event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                            new TrcPose2D(60.0, 0.0, 0.0));
                        sm.addEvent(event);
                        sm.waitForEvents(State.DESCEND, false);
                    }
                    else {
                        sm.setState(State.DONE);
                    }
                    break;

                case DESCEND:
                    if (tiltEvent.isSignaled()) {
                        event.clear();
                        sm.addEvent(tiltEvent);
                        robot.robotDrive.purePursuitDrive.start(
                            event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                            new TrcPose2D(60.0, 0.0, 0.0));
                        sm.addEvent(event);
                        sm.waitForEvents(State.DESCEND, false);
                    }
                    else {
                        sm.setState(State.DONE);
                    }
                    break;

                case EXIT:
                    tiltEvent.clear();
                    robot.robotDrive.purePursuitDrive.cancel();
                    sm.setState(State.BALANCE);
                    break;

                case BALANCE:
                    // TODO: Determine which direction robot will strafe onto charging station
                    robot.autoBalanceTask.autoAssistBalance(BalanceStrafeDir.LEFT, event);
                    sm.waitForSingleEvent(event, State.DONE);
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

}   //class CmdAutoStartPos2
