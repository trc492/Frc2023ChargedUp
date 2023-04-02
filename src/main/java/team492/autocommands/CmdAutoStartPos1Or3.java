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
        //drives to a point where it can use autopickup to pickup the object 
        DRIVE_TO_LOOK_POS,
        GET_SECOND,
        DRIVE_TO_SCORE,
        SCORE_SECOND_OBJ,
        // DRIVE_TO_BALANCE,
        // BALANCE,
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
    private ObjectType preloadType = ObjectType.CONE;
    private int scoreLevel = 0;
    private ScoreLocation scoreLocation = ScoreLocation.RIGHT;
    private boolean doAutoBalance = false;
    private boolean scoreSecondPiece = true;
    //hardcoded for now 
    private int secondPieceLevel = 2; 
    private ScoreLocation secondObjScoreLoc = ScoreLocation.MIDDLE; 
    private ObjectType secondObjType = ObjectType.CUBE; 
    private double timeToScore = 3.0; 
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
                    preloadType = FrcAuto.autoChoices.getPreloadedObjType();
                    scoreLevel = FrcAuto.autoChoices.getScoreLevel();
                    scoreLocation = FrcAuto.autoChoices.getScoreLocation();
                    // TODO: add autoBalance functionality
                    // TODO: add option to score another element
                    doAutoBalance = FrcAuto.autoChoices.getDoAutoBalance();
                    scoreSecondPiece = FrcAuto.autoChoices.getScoreSecondPiece();

                    // Set robot's absolute field position according to the start position in autoChoices.
                    robot.robotDrive.setFieldPosition(null, false);

                    if (scorePreload && scoreLevel == 0)
                    {
                        robot.armPidActuator.setPosition(
                            moduleName, 0.0, RobotParams.ARM_LOW_POS, true, RobotParams.ARM_MAX_POWER, null, 0.0);
                        // Deploying & spinning intake to score the preloaded object to ground level.
                        if (preloadType == ObjectType.CONE)
                        {
                            robot.wristPidActuator.setPosition(moduleName, 0.0, 20.0, true, 1.0, null, 0.0);
                        }
                        robot.intake.setPower(moduleName, 0.5, preloadType == ObjectType.CONE? RobotParams.INTAKE_CONE_SPIT_POWER: RobotParams.INTAKE_CUBE_SPIT_POWER, 0.5, intakeEvent);
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
                    sm.waitForSingleEvent(driveEvent, (scorePreload && scoreLevel > 0)? State.SCORE_PRELOAD_HIGH: State.EXIT_COMMUNITY);
                    break;

                case SCORE_PRELOAD_HIGH:
                    // Call autoScore to score the object.
                    robot.autoScoreTask.autoAssistScoreObject(
                        preloadType, scoreLevel, scoreLocation, false, false, autoAssistEvent);
                    State nextState = scoreSecondPiece? State.GET_SECOND: State.EXIT_COMMUNITY; 
                    sm.waitForSingleEvent(autoAssistEvent, nextState);
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
                    // doAutoBalance is hard coded to false, so we will not do balance. When we are ready to balance,
                    // just remove the hard code.
                    sm.waitForSingleEvent(driveEvent, State.DONE);
                    // robot.robotDrive.enableDistanceTrigger(Math.sqrt(169.0 + xOffset*xOffset), driveEvent);
                    // robot.robotDrive.driveBase.holonomicDrive(
                    //     null, xOffset/120.0, alliance == Alliance.Blue? 0.3: -0.3, 0.0, robot.robotDrive.driveBase.getHeading());
                    
                    break;
                
                //drives to a position behind the game object so in the next state we can use vision to go pickup 
                case DRIVE_TO_LOOK_POS:
                    if (startPos == 0 && alliance == Alliance.Blue || startPos == 2 && alliance == Alliance.Red)
                    {
                        // We are going for the game piece on the guardrail side. (BLUE RIGHT)
                        robot.robotDrive.purePursuitDrive.start(
                            driveEvent, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.adjustPosByAlliance(
                                alliance,
                                new TrcPose2D(RobotParams.CENTER_BETWEEN_CHARGING_STATION_AND_FIELD_EDGE_X,
                                            RobotParams.CHARGING_STATION_CENTER_BLUE_Y, 180.0)),
                            robot.robotDrive.adjustPosByAlliance(
                                alliance,
                                new TrcPose2D(RobotParams.GAME_PIECE_1_X, RobotParams.GAME_PIECE_BLUE_Y - 36.0, 0.0)));
                    }
                    else
                    {
                        // We are going for the game piece on the substation side. (BLUE LEFT)
                        robot.robotDrive.purePursuitDrive.start(
                            driveEvent, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.adjustPosByAlliance(
                                alliance,
                                new TrcPose2D(-185,
                                            RobotParams.CHARGING_STATION_CENTER_BLUE_Y, 180.0)),
                            robot.robotDrive.adjustPosByAlliance(
                                alliance,
                                new TrcPose2D(RobotParams.GAME_PIECE_4_X, RobotParams.GAME_PIECE_BLUE_Y - 36, 0.0)))
                            ;
                            
                    }
                    sm.waitForSingleEvent(driveEvent, State.GET_SECOND);
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
                //              robot.autoPickupTask.autoAssistPickup(ObjectType.CONE, true, doAutoBalance, autoAssistEvent);
                //     //     }
                //     // }, null);
                //     // sm.waitForSingleEvent(autoAssistEvent, State.DONE);//DRIVE_TO_SCORE);
                //     break;
                //picks up second object 

                //use autoassist pickup to drive to and pickup the object 
                case GET_SECOND:
                    robot.autoPickupTask.autoAssistPickup(secondObjType, true, autoAssistEvent);
                    sm.waitForSingleEvent(autoAssistEvent, State.DRIVE_TO_SCORE);
                    break; 
                
                case DRIVE_TO_SCORE:
                    //do turtle mode so vision can see the target
                    if(robot.intake.hasObject()){
                        robot.turtleMode(moduleName);
                        //drive to a place where we can see the apriltag
                        if (startPos == 0 && alliance == Alliance.Blue || startPos == 2 && alliance == Alliance.Red)
                        {
                            // We are scoring on the far right cube node - BLUE RIGHT
                            robot.robotDrive.purePursuitDrive.start(
                                driveEvent, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.adjustPosByAlliance(
                                    alliance,
                                    new TrcPose2D(RobotParams.CENTER_BETWEEN_CHARGING_STATION_AND_FIELD_EDGE_X,
                                                220, 180)),
                                robot.robotDrive.adjustPosByAlliance(
                                    alliance,
                                    new TrcPose2D(RobotParams.CENTER_BETWEEN_CHARGING_STATION_AND_FIELD_EDGE_X, 85, 180.0)));
                        }
                        else
                        {
                            // We are going for the game piece on the substation side. (BLUE LEFT)
                            robot.robotDrive.purePursuitDrive.start(
                                driveEvent, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.adjustPosByAlliance(
                                    alliance,
                                    new TrcPose2D(-181,
                                                220, 180.0)),
                                robot.robotDrive.adjustPosByAlliance(
                                    alliance,
                                    new TrcPose2D(-181, 85, 180.0)))
                                ;
                                
                        }
                        sm.waitForSingleEvent(driveEvent, State.SCORE_SECOND_OBJ);
                    }
                    else{
                        sm.setState(State.DONE);
                    }
                    
                    
                    break;
                case SCORE_SECOND_OBJ: 
                    //check match time to decide if we can score
                    if(15 - elapsedTime > timeToScore ){
                        robot.autoScoreTask.autoAssistScoreObject(secondObjType, secondPieceLevel, ScoreLocation.MIDDLE, true, false, autoAssistEvent);
                        sm.waitForSingleEvent(autoAssistEvent, State.DONE);
                    }
                    else{
                        sm.setState(State.DONE); 
                    }

                    
                    break; 

                // case DRIVE_TO_BALANCE:

                //     TrcPose2D balancePose;
                //     if (alliance == Alliance.Blue)
                //     {
                //         balancePose = new TrcPose2D(
                //             RobotParams.STARTPOS_2_X, RobotParams.STARTPOS_BLUE_Y + 144.0, -90.0);
                //     }
                //     else
                //     {
                //         balancePose = new TrcPose2D(
                //             RobotParams.STARTPOS_2_X, RobotParams.STARTPOS_RED_Y - 144.0, 90.0);
                //     }

                //     robot.robotDrive.purePursuitDrive.start(
                //         driveEvent, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                //         balancePose);
                //     sm.waitForSingleEvent(driveEvent, State.BALANCE);
                //     break;

                // case BALANCE:
                //     // We're now next to the station outside of community, so we can do autobalance!
                //     robot.autoBalanceTask.autoAssistBalance(BalanceInitSide.OUTSIDE, autoAssistEvent);
                //     sm.waitForSingleEvent(autoAssistEvent, State.DONE);
                    // break;

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
