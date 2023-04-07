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
import team492.FrcAuto.BalanceInitSide;
import team492.FrcAuto.ObjectType;
import team492.FrcAuto.ScoreLocation;
import team492.drivebases.SwerveDrive.TiltDir;

public class CmdAutoStartPos2 implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoStartPos2";

    private enum State
    {
        START,
        PREP_TO_BALANCE,
        START_TO_CLIMB,
        CLIMB,
        LEVEL,
        DESCEND,
        BALANCE,
        DONE
    }   //enum State


    private final Robot robot;
    private final TrcEvent driveEvent;
    private final TrcEvent autoAssistEvent;
    private final TrcEvent tiltEvent;
    private final TrcEvent distanceEvent;
    private final TrcStateMachine<State> sm;

    // TODO: Test all iterations to verify State shenanigans
    private Alliance alliance = Alliance.Blue;
    private boolean scorePreload = true;
    private ObjectType preloadType = ObjectType.CUBE;
    private int scoreLevel = 2;
    private ScoreLocation scoreLocation = ScoreLocation.MIDDLE;
    private boolean doAutoBalance = true;
    private TrcPose2D startPos;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     */
    public CmdAutoStartPos2(Robot robot)
    {
        this.robot = robot;
        driveEvent = new TrcEvent(moduleName + ".driveEvent");
        autoAssistEvent = new TrcEvent(moduleName + ".autoAssistEvent");
        tiltEvent = new TrcEvent(moduleName + ".tiltEvent");
        distanceEvent = new TrcEvent(moduleName + ".distanceEvent");
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
        robot.robotDrive.cancel();
        robot.robotDrive.purePursuitDrive.setMoveOutputLimit(RobotParams.PPD_MOVE_DEF_OUTPUT_LIMIT);
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

            robot.dashboard.displayPrintf(8, "State: %s", state);
            robot.globalTracer.traceInfo(
                moduleName, "[%.3f] %s: xDist=%.1f, tilt=%.3f, enteringBalance=%s, exitingBalance=%s, tiltTriggered=%s",
                TrcTimer.getModeElapsedTime(), state, robot.robotDrive.driveBase.getXPosition(), tiltAngle,
                enterBalance, exitBalance, tiltTriggered);

            switch (state)
            {
                case START:
                    // Read autoChoices.
                    alliance = FrcAuto.autoChoices.getAlliance();
                    scorePreload = FrcAuto.autoChoices.getScorePreload();
                    preloadType = FrcAuto.autoChoices.getPreloadedObjType();
                    scoreLevel = FrcAuto.autoChoices.getScoreLevel();
                    scoreLocation = FrcAuto.autoChoices.getScoreLocation();
                    doAutoBalance = FrcAuto.autoChoices.getDoAutoBalance();
                    startPos = alliance == Alliance.Blue? RobotParams.STARTPOS_BLUE_2: RobotParams.STARTPOS_RED_2;
                    // Set robot's absolute field position according to the start position in autoChoices.
                    robot.robotDrive.setFieldPosition(startPos, false);
                    robot.wristPidActuator.setPosition(RobotParams.WRIST_MIN_POS, true);

                    // Zero elevator (just in case)
                    // TODO: Remove if unnecessary
                    robot.elevatorPidActuator.zeroCalibrate();

                    if (scorePreload)
                    {
                        robot.autoScoreTask.autoAssistScoreObject(
                            preloadType, scoreLevel, scoreLocation, false, false, autoAssistEvent);
                        sm.waitForSingleEvent(autoAssistEvent, doAutoBalance? State.PREP_TO_BALANCE: State.DONE);
                    }
                    else
                    {
                        sm.setState(doAutoBalance? State.PREP_TO_BALANCE: State.DONE);
                    }
                    break;

                case PREP_TO_BALANCE:
                    // Back up and turn right to prepare to crab over the station.
                    robot.robotDrive.purePursuitDrive.start(
                        driveEvent, 1.5, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(0.0, -36.0, 90.0));
                    sm.waitForSingleEvent(driveEvent, State.START_TO_CLIMB);
                    break;

                case START_TO_CLIMB:
                    // Start climbing the charging station and enable tilt trigger to monitor different climbing stages.
                    robot.robotDrive.enableTiltTrigger(tiltEvent);
                    robot.robotDrive.driveBase.holonomicDrive(
                        null, 0.0, alliance == Alliance.Blue? 0.3: -0.3, 0.0, robot.robotDrive.driveBase.getHeading());
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
                        robot.robotDrive.enableDistanceTrigger(6.0, distanceEvent);
                        sm.waitForSingleEvent(distanceEvent, State.BALANCE);
                    }
                    else
                    {
                        sm.waitForSingleEvent(tiltEvent, State.DESCEND);
                    }
                    break;

                case BALANCE:
                    // We're now next to the station outside of community, so we can do autobalance!
                    robot.robotDrive.driveBase.stop();
                    robot.robotDrive.disableDistanceTrigger();
                    robot.robotDrive.disableTiltTrigger();
                    if (doAutoBalance)
                    {
                        robot.autoBalanceTask.autoAssistBalance(BalanceInitSide.OUTSIDE, autoAssistEvent);
                        sm.waitForSingleEvent(autoAssistEvent, State.DONE);
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
