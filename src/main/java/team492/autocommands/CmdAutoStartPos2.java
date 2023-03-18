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
import team492.FrcAuto;
import team492.Robot;
import team492.RobotParams;
import team492.FrcAuto.BalanceStrafeDir;
import team492.FrcAuto.ObjectType;
import team492.FrcAuto.ScoreLocation;
import team492.drivebases.SwerveDrive.TiltDir;

public class CmdAutoStartPos2 implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoStartPos2";

    private enum State
    {
        START,
        UNTUCK_ARM,
        SCORE_PRELOAD,
        TURN,
        START_TO_CLIMB,
        CLIMB,
        LEVEL,
        DESCEND,
        GO_BALANCE,
        DONE
    }   //enum State


    private final Robot robot;
    private final TrcEvent event;
    private final TrcEvent tiltEvent;
    private final TrcStateMachine<State> sm;

    private int scoreLevel = 0;
    private boolean scorePreload = true;
    private boolean doAutoBalance = true;
    private boolean untuck = true;

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
        sm.start(State.START);
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
            double tiltAngle = robot.robotDrive.getGyroRoll();
            TiltDir enterBalance = robot.robotDrive.enteringBalanceZone();
            TiltDir exitBalance = robot.robotDrive.exitingBalanceZone();
            boolean tiltTriggered = tiltEvent.isSignaled();
            State nextState;

            robot.dashboard.displayPrintf(8, "State: %s", state);
            robot.globalTracer.traceInfo(
                moduleName, "[%.3f] %s: xDist=%.1f, tilt=%.3f, enteringBalance=%s, exitingBalance=%s, tiltTriggered=%s",
                TrcTimer.getModeElapsedTime(), state, robot.robotDrive.driveBase.getXPosition(), tiltAngle,
                enterBalance, exitBalance, tiltTriggered);

            switch (state)
            {
                case START:
                    // Read autoChoices.
                    // scoreLevel = FrcAuto.autoChoices.getScoreLevel();
                    // scorePreload = FrcAuto.autoChoices.getScorePreload();
                    // doAutoBalance = FrcAuto.autoChoices.getDoAutoBalance();

                    // Set robot's absolute field position according to the start position in autoChoices.
                    robot.robotDrive.setFieldPosition(null, false);
                    if (untuck)
                    {
                        // When the arm is tucked in, the elevator is raised with a pre-determined offset. Since we
                        // can't zero calibrate, set the elevator to that offset.
                        // robot.elevator.setAutoStartOffset(RobotParams.ELEVATOR_AUTOSTART_OFFSET);                        
                    }

                    // Back up a little so autoScore can raise the arm without hitting the shelf, and signal event when done.
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.5);


                    if (scorePreload)
                    {
                        if (scoreLevel == 0)
                        {
                            robot.intake.extend(0.05);
                            robot.intake.setPower(0.2, RobotParams.INTAKE_SPIT_POWER, RobotParams.INTAKE_SPIT_POWER, 0.5);
                            nextState = State.UNTUCK_ARM;
                            robot.robotDrive.purePursuitDrive.start(
                                event, 1.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                                new TrcPose2D(0.0, -24.0, 0.0));
                        }
                        else
                        {
                            // Deploy the intake so the arm can come out.
                            robot.intake.extend(0.05);
                            // Depoly the arm by raising elevator and untuck the arm (fire and forget).
                            robot.elevatorPidActuator.setPosition(
                                RobotParams.ELEVATOR_SAFE_HEIGHT, true, 1.0, null, 0.5);
                            robot.armPidActuator.setPosition(
                                null, 0.7, RobotParams.ARM_TRAVEL_POSITION, true, RobotParams.ARM_MAX_POWER,
                                null, 0.0);
                            nextState = State.SCORE_PRELOAD;
                        }
                    }
                    else
                    {
                        nextState = doAutoBalance? State.TURN: State.DONE;
                    }
                    
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case UNTUCK_ARM:
                    if (untuck)
                    {
                        // Just finished scoring at level 0, untuck the arm and prepare to turn.
                        // robot.elevatorPidActuator.setPosition(
                        //     RobotParams.ELEVATOR_SAFE_HEIGHT, true, 1.0, event, 0.5);
                        // robot.armPidActuator.setPosition(
                        //     null, 0.7, RobotParams.ARM_TRAVEL_POSITION, true, RobotParams.ARM_MAX_POWER,
                        //     null, 0.0);
                    }

                    robot.intake.retract(untuck? 0.8: 0.1);
                    robot.robotDrive.purePursuitDrive.start(
                        event, 4.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(0.0, -156.0, 0.0));
                    sm.waitForSingleEvent(event, doAutoBalance? State.TURN: State.DONE);
                    break;

                case SCORE_PRELOAD:
                    // Call autoScore to score the object.
                    robot.autoScoreTask.autoAssistScoreObject(
                        ObjectType.CUBE, scoreLevel, ScoreLocation.MIDDLE, false, event);
                    sm.waitForSingleEvent(event, State.TURN);
                    break;
                
                case TURN:
                    // Turn right to prepare to crab over the station.
                    robot.robotDrive.purePursuitDrive.start(
                        event, 2.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(0.0, 0.0, 90.0));
                    sm.waitForSingleEvent(event, State.START_TO_CLIMB);
                    break;

                case START_TO_CLIMB:
                    // Start climbing the charging station and enable tilt trigger to monitor different climbing stages.
                    robot.robotDrive.enableTiltTrigger(tiltEvent);
                    robot.robotDrive.driveBase.holonomicDrive(0.35, 0.0, 0.0);
                    sm.waitForSingleEvent(tiltEvent, State.CLIMB, 5.0);
                    break;
                
                case CLIMB:
                    // We're climbing up the station, going to the next state when we're level on the station.
                    if (tiltTriggered)
                    {
                        // When entering the balance zone, we are about to level off. If not, we are still climbing.
                        sm.waitForSingleEvent(
                            tiltEvent,
                            enterBalance != null && enterBalance == TiltDir.TILT_LEFT? State.LEVEL: State.CLIMB);
                    }
                    else
                    {
                        // We missed the platform because our angle hasn't changed.
                        sm.setState(State.DONE);
                    }
                    break;

                case LEVEL:
                    // When exiting the balance zone, we are descending the charging station. If not, keep waiting.
                    sm.waitForSingleEvent(
                        tiltEvent,
                        exitBalance != null && exitBalance == TiltDir.TILT_RIGHT? State.DESCEND: State.LEVEL);
                    break;

                case DESCEND:
                    // We are about leveling again which means we are getting on flat ground but we should run
                    // the robot a little longer to make sure it clears the charging station.
                    if (enterBalance != null && enterBalance == TiltDir.TILT_RIGHT)
                    {
                        robot.robotDrive.enableDistanceTrigger(2.0, event);
                        sm.waitForSingleEvent(event, State.GO_BALANCE);
                    }
                    else
                    {
                        sm.waitForSingleEvent(tiltEvent, State.DESCEND);
                    }
                    break;

                case GO_BALANCE:
                    // We're now next to the station outside of community, so we can do autobalance!
                    robot.robotDrive.driveBase.stop();
                    robot.robotDrive.disableDistanceTrigger();
                    robot.robotDrive.disableTiltTrigger();
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
