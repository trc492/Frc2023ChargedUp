package team492;

import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTimer;

class CmdAuto implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAuto";

    private enum State
    {
        SCORE,
        START_DELAY,
        GO_TO_GAME_PIECE,
        PICKUP_PIECE,
        GO_TO_SCORE_POSITION,
        GO_TO_PARK,
        PARK,   // AutoBalance
        DONE
        //States
    }   //enum State


    private final Robot robot;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;

    private int piecesScored = 0;
    private Boolean park = false; //TODO: connect with auto choices (i dont know how, it is false as a placeholder) 

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     */
    CmdAuto(Robot robot)
    {
        robot.globalTracer.traceInfo(
            moduleName, ">>> robot=%s, choices=%s", robot, FrcAuto.autoChoices);

        this.robot = robot;
        timer = new TrcTimer(moduleName + ".timer");
        event = new TrcEvent(moduleName + ".event");
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START_DELAY);
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
                case SCORE: //Scores a game piece, the precondition being that it is already in the scoring position, with a game piece in the robot, depending on if its a cone or cube
                    //TODO: implement code for scoring
                    piecesScored++;
                    if (piecesScored == 1) {
                        sm.waitForSingleEvent(event, State.START_DELAY);
                    }
                    else if (piecesScored ==2 && !park){
                        sm.waitForSingleEvent(event, State.GO_TO_GAME_PIECE);
                    }
                    else {
                        sm.waitForSingleEvent(event, State.GO_TO_PARK);
                    }
                
                    break;

                case START_DELAY:
                    double startDelay = FrcAuto.autoChoices.getStartDelay();
                    if (startDelay == 0.0) {
                        sm.setState(State.GO_TO_GAME_PIECE);
                    }
                    else {
                        sm.waitForSingleEvent(event, State.GO_TO_GAME_PIECE);
                        timer.set(startDelay, event);
                    }
                    break;

                case GO_TO_GAME_PIECE: //Drives to a few feet (3ft from the center of the robot to the ball) behind the game piece we want to pick up, determining the location by how many pieces we have already scored
                    if (piecesScored == 1) {
                        //drives to 3ft behind the right most piece from the init scoring pos
                        if (FrcAuto.autoChoices.getAlliance().equals("Blue")) {
                            robot.robotDrive.purePursuitDrive.start(
                                event, 2.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                new TrcPose2D(-33.0, 85.0, 45.0),
                                new TrcPose2D(-33.0, 238.0, 0.0));
                        }
                        else {
                            robot.robotDrive.purePursuitDrive.start(
                                event, 2.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                new TrcPose2D(-33.0, 564.0, 225.0),
                                new TrcPose2D(-33.0, 404.0, 180.0));
                        }
                    }
                    else if (piecesScored == 2) {
                        //drives to the location of the second right most piece from second scoring pos (shelf closest to scoring table)
                        if (FrcAuto.autoChoices.getAlliance().equals("Blue")) {
                            robot.robotDrive.purePursuitDrive.start(
                                event, 2.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                new TrcPose2D(-33.0, 85.0, 45.0),
                                new TrcPose2D(-33.0, 220.0, 0.0),
                                new TrcPose2D(-82.0, 238.0, 0.0));

                        }
                        else {
                            robot.robotDrive.purePursuitDrive.start(
                                event, 2.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                                new TrcPose2D(-33.0, 564.0, 225.0),
                                new TrcPose2D(-33.0, 429.0, 180.0),
                                new TrcPose2D(-82.0, 404.0, 180.0));
                        }
                    }
                    sm.waitForSingleEvent(event, State.PICKUP_PIECE);
                    break;
                    
                case PICKUP_PIECE: //Drives forward while running intake until it picks up a game piece, the precondition being that we already at the correct location
                    //runs intake
                    //drives forward
                    //stops running intake when piece detected in robot and broadcasts event
                    sm.waitForSingleEvent(event, State.GO_TO_SCORE_POSITION);
                    break;

                case GO_TO_SCORE_POSITION: //Drives to the scoring position, determining the location based on how many pieces we have already scored
                    if (piecesScored == 1) {
                        //drives to the location of the right most piece, broadcasting event when pp finishes
                        if (FrcAuto.autoChoices.getAlliance().equals("Blue")) {

                        }
                        else {
                            
                        }
                    }
                    else if (piecesScored == 2) {
                    //drives to the location of the second right most piece, broadcasting event when pp finishes
                        if (FrcAuto.autoChoices.getAlliance().equals("Blue")) {

                        }
                        else {
                        
                        }
                    }
                    sm.waitForSingleEvent(event, State.SCORE);
                    break;

                case GO_TO_PARK: //Drives to the location to park
                    //TODO: implement code for driving to park location, broadcasting event when pp finishes
                    if (FrcAuto.autoChoices.getAlliance().equals("Blue")) {

                    }
                    else {
                        
                    }
                    sm.waitForSingleEvent(event, State.PARK);
                    break;

                case PARK: //Parks
                    //TODO: implement code for auto parking, broadcasting event when done
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
