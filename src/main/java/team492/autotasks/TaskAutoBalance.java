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
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 package team492.autotasks;

import TrcCommonLib.trclib.TrcAutoTask;
import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcRobot.RunMode;
import TrcCommonLib.trclib.TrcTaskMgr.TaskType;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcTimer;
import team492.Robot;
import team492.FrcAuto.BalanceStrafeDir;

/**
 * This class implements auto-assist balancing on the charging station.
 *
 * Preconditions:
 * - Robot is on flat ground in front of the charging station.
 * - Robot is facing sideways so that its edge is square with the charging station
 */
public class TaskAutoBalance extends TrcAutoTask<TaskAutoBalance.State>
{
    private static final String moduleName = "TaskAutoBalance";

    public enum State
    {
        START,
        CLIMB,
        SETTLE,
        CHECK,
        DONE
    }   //enum State

    private final String owner;
    private final Robot robot;
    private final TrcDbgTrace msgTracer;
    private final TrcEvent event, tiltEvent;
    private final TrcTimer timer;
    private String currOwner = null;
    private double startDir;
    private boolean correcting = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param owner specifies the owner name to take subsystem ownership, can be null if no ownership required.
     * @param robot specifies the robot object that contains all the necessary subsystems.
     * @param msgTracer specifies the tracer to use to log events, can be null if not provided.
     */
    public TaskAutoBalance(String owner, Robot robot, TrcDbgTrace msgTracer)
    {
        super(moduleName, owner, TrcTaskMgr.TaskType.POST_PERIODIC_TASK, msgTracer);
        this.owner = owner;
        this.robot = robot;
        this.msgTracer = msgTracer;
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        tiltEvent = new TrcEvent(moduleName + ".tiltEvent");
    }   //TaskAutoBalance

    /**
     * This method starts the auto-assist task to climb and balance on the charging station.
     *
     * @param strafeDir specifies the direction to strafe up the charging station.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoAssistBalance(BalanceStrafeDir strafeDir, TrcEvent completionEvent)
    {
        final String funcName = "autoAssistBalance";

        if (msgTracer != null)
        {
            msgTracer.traceInfo( funcName, "%s: strafeDir=%s, event=%s", moduleName, strafeDir, completionEvent);
        }

        startDir = strafeDir == BalanceStrafeDir.LEFT? -1.0: 1.0;
        startAutoTask(State.START, null, completionEvent);
    }   //autoAssistBalance

    /**
     * This method cancels an in progress auto-assist operation if any.
     */
    public void autoAssistCancel()
    {
        final String funcName = "autoAssistCancel";

        if (msgTracer != null)
        {
            msgTracer.traceInfo( funcName, "%s: canceled.", moduleName);
        }

        stopAutoTask(false);
    }   //autoAssistCancel

    //
    // Implement TrcAutoTask abstract methods.
    //

    /**
     * This method is called by the super class to acquire ownership of all subsystems involved in the auto-assist
     * operation. This is typically done before starting an auto-assist operation.
     *
     * @return true if acquired all subsystems ownership, false otherwise. It releases all ownership if any acquire
     *         failed.
     */
    @Override
    protected boolean acquireSubsystemsOwnership()
    {
        boolean success = owner == null || robot.robotDrive.driveBase.acquireExclusiveAccess(owner);

        if (success)
        {
            currOwner = owner;
        }
        else
        {
            releaseSubsystemsOwnership();
        }

        return success;
    }   //acquireSubsystemsOwnership

    /**
     * This method is called by the super class to release ownership of all subsystems involved in the auto-assist
     * operation. This is typically done if the auto-assist operation is completed or canceled.
     */
    @Override
    protected void releaseSubsystemsOwnership()
    {
        if(owner != null)
        {
            robot.robotDrive.driveBase.releaseExclusiveAccess(currOwner);
            currOwner = null;
        }
    }   //releaseSubsystemsOwnership

    /**
     * This method is called by the super class to stop all the subsystems.
     */
    @Override
    protected void stopSubsystems()
    {
        robot.robotDrive.cancel(currOwner);
    }   //stopSubsystems

    /**
     * This methods is called periodically to run the auto-assist task.
     *
     * @param params specifies the task parameters.
     * @param state specifies the current state of the task.
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    @Override
    protected void runTaskState(
        Object params, State state, TaskType taskType, RunMode runMode, boolean slowPeriodicLoop)
    {
        double tiltAngle = robot.robotDrive.getGyroRoll();
        double dir = -Math.signum(tiltAngle);
        boolean inBalance = robot.robotDrive.inBalanceZone();
        boolean leveling = robot.robotDrive.startingToLevel();
        boolean tiltSignaled = tiltEvent.isSignaled();

        if (msgTracer != null)
        {
            msgTracer.traceInfo(
                moduleName, "[%.3f] %s: xDist=%.1f, tilt=%.3f, inBalance=%s, leveling=%s, tiltSignaled=%s",
                TrcTimer.getModeElapsedTime(), state, robot.robotDrive.driveBase.getXPosition(), tiltAngle,
                inBalance, leveling, tiltSignaled);
        }

        switch (state)
        {
            case START:
                // Arm tilt trigger to signal tiltEvent.
                // Strafe up the charging station slowly with a safety limit of 5 seconds.
                robot.robotDrive.enableTiltTrigger(tiltEvent);
                robot.robotDrive.driveBase.holonomicDrive(currOwner, startDir*0.25, 0.0, 0.0);
                sm.waitForSingleEvent(tiltEvent, State.CLIMB, 5.0);
                break;

            case CLIMB:
                if (tiltSignaled)
                {
                    if (leveling)
                    {
                        // We are starting to level off, drive a fixed distance to the center of the charging station.
                        // Arm a distance trigger to do this.
                        robot.robotDrive.enableDistanceTrigger(correcting? 12.0: 20.0, event);
                        sm.waitForSingleEvent(event, State.SETTLE);
                    }
                    else
                    {
                        // We are still going uphill, keep climbinig.
                        sm.waitForSingleEvent(tiltEvent, State.CLIMB);
                    }
                }
                else
                {
                    // We timed out. We somehow missed the charging station, call it quit.
                    sm.setState(State.DONE);
                }
                break;

            case SETTLE:
                // It takes time for the charging station to balance, wait for it to settle.
                robot.robotDrive.driveBase.stop(currOwner);
                timer.set(1.0, event);
                sm.waitForSingleEvent(event, State.CHECK);
                break;

            case CHECK:
                if (!inBalance)
                {
                    // Robot is tipped. Drive the robot to the climb direction.
                    correcting = true;
                    robot.robotDrive.driveBase.holonomicDrive(currOwner, dir*0.2, 0.0, 0.0);
                    sm.waitForSingleEvent(tiltEvent, State.CLIMB);
                }
                else
                {
                    // We are balanced, done.
                    sm.setState(State.DONE);
                }
                break;
            
            default:
            case DONE:
                robot.robotDrive.disableTiltTrigger();
                robot.robotDrive.disableDistanceTrigger();
                robot.robotDrive.driveBase.stop(currOwner);
                robot.robotDrive.setAntiDefenseEnabled(currOwner, true);
                stopAutoTask(true);
                break;
        }
    }   //runTaskState

}   //class TaskAutoBalance
