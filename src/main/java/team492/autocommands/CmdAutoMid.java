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

import TrcCommonLib.trclib.TrcDbgTrace;
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

public class CmdAutoMid implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoMid";
    private static final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();

    private enum State
    {
        START,
        SCORE_PRELOAD,
        TURN,
        EXIT_COMMUNITY,
        GET_ON_CHARGE,
        BALANCE,
        DONE

    }   //enum State


    private final Robot robot;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;

    private boolean balance;
    private boolean scorePreload;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     */
    public CmdAutoMid(Robot robot)
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
            globalTracer.traceInfo(moduleName, "State: disabled or waiting (nextState=%s)...", sm.getNextState());
        }
        else
        {
            State nextState;

            robot.dashboard.displayPrintf(8, "State: %s", state);
            globalTracer.traceInfo(moduleName, "State: %s", state);
            switch (state)
            {

                case START:
                    balance = FrcAuto.autoChoices.getDoAutoBalance();
                    scorePreload = FrcAuto.autoChoices.getScorePreload();

                    robot.robotDrive.setFieldPosition(null, false);
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.25);
                    if(scorePreload)
                    {
                        robot.elevatorPidActuator.setPosition(moduleName, 0.0, RobotParams.ELEVATOR_SAFE_HEIGHT, true, 1.0, null, 0.5);
                        robot.armPidActuator.setPosition(moduleName, 0.5, RobotParams.ARM_TRAVEL_POSITION, true, RobotParams.ARM_MAX_POWER, null, 0.5);
                        sm.setState(State.SCORE_PRELOAD);
                    }
                    sm.setState(State.TURN);
                    break;

                case SCORE_PRELOAD:
                    robot.autoScoreTask.autoAssistScoreObject(ObjectType.CUBE, 2, ScoreLocation.MIDDLE, false, event);
                    sm.waitForSingleEvent(event, State.TURN);
                    break;
                
                case TURN:

                    robot.robotDrive.purePursuitDrive.start(
                        event, 1.5, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(0.0, 0.0, 90.0));
                    sm.waitForSingleEvent(event, State.EXIT_COMMUNITY);
                    break;
                
                case EXIT_COMMUNITY:
                    robot.robotDrive.purePursuitDrive.start(
                        event, 7, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(180.0, 0.0, 0.0));
                    if(balance)
                    {
                        sm.waitForSingleEvent(event, State.GET_ON_CHARGE);
                    }
                    else
                    {
                        sm.setState(State.DONE);
                    }
                    break;
                
                case GET_ON_CHARGE:
                    robot.robotDrive.purePursuitDrive.start(
                        event, 5, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(-60.0, 0.0, 0.0));
                    sm.waitForSingleEvent(event, State.BALANCE);
                    break;

                case BALANCE:
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
