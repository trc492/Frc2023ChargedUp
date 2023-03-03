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
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcTaskMgr.TaskType;
import team492.Robot;

/**
 * This class implements auto-assist balancing on the charging station.
 *
 * Preconditions:
 * - Robot is on flat ground in front of the charging station.
 * - Robot is facing sideways so that its right edge is square with the charging station
 */
public class TaskAutoBalance extends TrcAutoTask<TaskAutoBalance.State>
{
    private static final String moduleName = "TaskAutoBalance";

    public enum State
    {
        CLIMBING,
        // BALANCING,
        DONE
    }
    
    private final String owner;
    private final Robot robot;
    private final TrcDbgTrace msgTracer;
    private String currOwner = null;
    private double prevTilt;

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
    }   //TaskAutoBalance

    /**
     * This method starts the auto-assist task to climb and balance on the charging station.
     *
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoAssistBalance(TrcEvent completionEvent)
    {
        prevTilt = 0.0;
        startAutoTask(State.CLIMBING, null, completionEvent);
    }   //autoAssistBalance

    /**
     * This method cancels an in progress auto-assist operation if any.
     */
    public void autoAssistCancel()
    {
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
        // robot.robotDrive.balancePidDrive.cancel(currOwner);
        robot.robotDrive.driveBase.stop(currOwner);
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
        switch (state)
        {
            case CLIMBING:
                double currTilt = Math.abs(robot.robotDrive.getGyroRoll());

                if (msgTracer != null)
                {
                    msgTracer.traceInfo(moduleName, "prevTilt=%.2f, currTilt=%.2f", prevTilt, currTilt);
                }
                robot.dashboard.displayPrintf(
                    15, "Balance: prevTilt=%.2f, currTilt=%.2f", prevTilt, currTilt);
                robot.robotDrive.driveBase.holonomicDrive(currOwner, 0.25, 0.0, 0.0);
                // Check if our angle is decerasing (approaching zero)
                if (currTilt < prevTilt)
                {
                    robot.robotDrive.driveBase.stop(currOwner);
                    sm.setState(State.DONE);
                }
                else
                {
                    prevTilt = currTilt;
                }
                // // Move forward slowly until the robot is tipping backward.
                // // CodeReview: should not use Math.abs, just check the sign when it's tilting backward.
                // if (Math.abs(robot.robotDrive.getGyroPitch()) < 2.0)
                // {
                //     robot.robotDrive.driveBase.holonomicDrive(currOwner, 0.0, 0.2, 0.0);
                // }
                // else
                // {
                //     robot.robotDrive.driveBase.stop(currOwner);
                //     sm.setState(State.BALANCING);
                // }
                break;

            // case BALANCING:
            //     msgTracer.traceInfo(moduleName, "Balancing.");
            //     robot.robotDrive.balancePidDrive.setSensorTarget(
            //         currOwner, robot.robotDrive.driveBase.getXPosition(), 0.0,
            //         robot.robotDrive.driveBase.getHeading(), true, event, 0.0);
            //     sm.waitForSingleEvent(event, State.DONE);
            //     break;

            case DONE:
            default:
                robot.robotDrive.setAntiDefenseEnabled(currOwner, true);
                stopAutoTask(true);
                break;
        }
    }   //runTaskState

}   //class TaskAutoBalance
