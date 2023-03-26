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
        SCORE_PRELOAD_HIGH,
        EXIT_COMMUNITY,
        // GET_SECOND,
        // DRIVE_TO_SCORE,
        DRIVE_TO_BALANCE,
        BALANCE,
        DONE
    }   //enum State


    private final Robot robot;
    private final TrcEvent driveEvent;
    private final TrcEvent elevatorEvent;
    private final TrcEvent autoAssistEvent;
    private final TrcEvent intakeEvent;
    private final TrcStateMachine<State> sm;

    private Alliance alliance = Alliance.Blue;
    private int startPos = 2;
    private boolean scorePreload = true;
    private int scoreLevel = 0;
    private boolean doAutoBalance = false;
    // private boolean scoreSecondPiece = true;
    // private int piecesScored = 0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     */
    public CmdAutoStartPos1Or3(Robot robot)
    {
        this.robot = robot;
        driveEvent = new TrcEvent(moduleName + ".driveEvent");
        elevatorEvent = new TrcEvent(moduleName + ".elevatorEvent");
        autoAssistEvent = new TrcEvent(moduleName + ".autoAssistEvent");
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
            robot.dashboard.displayPrintf(8, "State: %s", state);
            switch (state)
            {
                case START:
                    // Read autoChoices.
                    alliance = FrcAuto.autoChoices.getAlliance();
                    startPos = FrcAuto.autoChoices.getStartPos();   // 0, 1, or 2.
                    scorePreload = FrcAuto.autoChoices.getScorePreload();
                    scoreLevel = FrcAuto.autoChoices.getScoreLevel();
                    // TODO: add autoBalance functionality
                    // doAutoBalance = FrcAuto.autoChoices.getDoAutoBalance();
                    // TODO: add option to score another element
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
                        // piecesScored++;
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
                    break;

                case UNTUCK_ARM:
                    robot.intake.extend();
                    robot.elevatorPidActuator.setPosition(
                        RobotParams.ELEVATOR_SAFE_HEIGHT, true, 1.0, elevatorEvent, 0.5);
                    robot.armPidActuator.setPosition(
                        null, 0.7, RobotParams.ARM_TRAVEL_POSITION, true, RobotParams.ARM_MAX_POWER,
                        null, 0.0);
                    robot.intake.retract(0.9);
                    sm.waitForSingleEvent(
                        elevatorEvent, scorePreload && scoreLevel > 0? State.SCORE_PRELOAD_HIGH: State.EXIT_COMMUNITY);
                    break;

                case SCORE_PRELOAD_HIGH:
                    // Call autoScore to score the object.
                    robot.autoScoreTask.autoAssistScoreObject(
                        ObjectType.CUBE, scoreLevel, ScoreLocation.MIDDLE, false, autoAssistEvent);
                    sm.waitForSingleEvent(autoAssistEvent, State.EXIT_COMMUNITY);
                    // piecesScored++;
                    break;

                case EXIT_COMMUNITY:
                    double xOffset = 0.0;
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
                    // robot.robotDrive.enableDistanceTrigger(Math.sqrt(169.0 + xOffset*xOffset), driveEvent);
                    // robot.robotDrive.driveBase.holonomicDrive(
                    //     null, xOffset/120.0, alliance == Alliance.Blue? 0.3: -0.3, 0.0, robot.robotDrive.driveBase.getHeading());
                    sm.waitForSingleEvent(driveEvent, State.DONE);
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
