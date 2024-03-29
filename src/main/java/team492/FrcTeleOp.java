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

package team492;

import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcRobot.RunMode;
import TrcFrcLib.frclib.FrcJoystick;
import TrcFrcLib.frclib.FrcXboxController;
import team492.FrcAuto.ObjectType;
import team492.drivebases.RobotDrive;

/**
 * This class implements the code to run in TeleOp Mode.
 */
public class FrcTeleOp implements TrcRobot.RobotMode
{
    private static final String moduleName = "FrcTeleOp";
    private static final boolean traceButtonEvents = true;
    //
    // Global objects.
    //
    protected final Robot robot;
    private boolean controlsEnabled = false;

    private boolean manualOverride = false;
    private boolean armControl = false;
    private boolean wristControl = false;
    private boolean spitting = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object to access all robot hardware and subsystems.
     */
    public FrcTeleOp(Robot robot)
    {
        //
        // Create and initialize global object.
        //
        this.robot = robot;
    }   //FrcTeleOp

    //
    // Implements TrcRobot.RunMode interface.
    //

    /**
     * This method is called when the teleop mode is about to start. Typically, you put code that will prepare
     * the robot for start of teleop here such as creating and configuring joysticks and other subsystems.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(RunMode prevMode, RunMode nextMode)
    {
        //
        // Enabling joysticks.
        //
        setControlsEnabled(true);
        //
        // Initialize subsystems for TeleOp mode if necessary.
        //
        if (robot.robotDrive != null)
        {
            robot.robotDrive.setDriveOrientation(RobotDrive.DriveOrientation.FIELD);
            robot.robotDrive.driveSpeedScale = RobotParams.DRIVE_MEDIUM_SCALE;
            robot.robotDrive.turnSpeedScale = RobotParams.TURN_MEDIUM_SCALE;
        }

        if (robot.elevator != null)
        {
            // TODO: Add back
            // robot.elevatorPidActuator.zeroCalibrate(moduleName);
        }
    }   //startMode

    /**
     * This method is called when teleop mode is about to end. Typically, you put code that will do clean
     * up here such as disabling joysticks and other subsystems.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void stopMode(RunMode prevMode, RunMode nextMode)
    {
        //
        // Disabling joysticks.
        //
        setControlsEnabled(false);
        //
        // Disable subsystems before exiting if necessary.
        //
        releaseAutoAssistAndSubsystems();
    }   //stopMode

    /**
     * This method is called periodically on the main robot thread. Typically, you put TeleOp control code here that
     * doesn't require frequent update For example, TeleOp joystick code or status display code can be put here since
     * human responses are considered slow.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    @Override
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
        if (slowPeriodicLoop)
        {
            if (controlsEnabled)
            {
                //
                // DriveBase operation.
                //
                if (robot.robotDrive != null)
                {
                    double[] inputs = robot.robotDrive.getDriveInputs();
                    double turnPower = inputs[2];
                    if (robot.driverController.getLeftTriggerAxis() > 0)
                    {
                        turnPower = 0.05;
                    }
                    else if (robot.driverController.getRightTriggerAxis() > 0)
                    {
                        turnPower = -0.05;
                    }

                    if (robot.robotDrive.driveBase.supportsHolonomicDrive())
                    {
                        robot.robotDrive.driveBase.holonomicDrive(
                            null, inputs[0], inputs[1], turnPower, getDriveGyroAngle());
                    }
                    else
                    {
                        robot.robotDrive.driveBase.arcadeDrive(inputs[1], inputs[2]);
                    }
                    robot.robotDrive.displaySteerEncoders(1);
                }
                //
                // Analog control of subsystem is done here if necessary.
                //
                if (RobotParams.Preferences.useSubsystems)
                {
                    double power = robot.operatorStick.getYWithDeadband(true);

                    if (robot.arm != null && armControl)
                    {
                        if (manualOverride)
                        {
                            robot.armPidActuator.setPower(power * RobotParams.ARM_MAX_POWER);
                        }
                        else
                        {
                            robot.armPidActuator.setPidPower(power * RobotParams.ARM_MAX_POWER, true);
                        }
                    }
                    else if (robot.wrist != null && wristControl)
                    {
                        if (manualOverride)
                        {
                            robot.wristPidActuator.setPower(-power * RobotParams.WRIST_MAX_POWER);
                        }
                        else
                        {
                            robot.wristPidActuator.setPidPower(-power * RobotParams.WRIST_MAX_POWER, true);
                        }
                    }
                    else if (robot.elevator != null)
                    {
                        if (manualOverride)
                        {
                            robot.elevatorPidActuator.setPower(power);
                        }
                        else
                        {
                            robot.elevatorPidActuator.setPidPower(power, true);
                        }
                    }
                }
            }
            //
            // Update robot status.
            //
            if (RobotParams.Preferences.doStatusUpdate)
            {
                robot.updateStatus();
            }
        }
    }   //periodic

    /**
     * This method enables/disables joystick controls.
     *
     * @param enabled specifies true to enable joystick control, false to disable.
     */
    protected void setControlsEnabled(boolean enabled)
    {
        controlsEnabled = enabled;

        if (RobotParams.Preferences.useDriverXboxController)
        {
            robot.driverController.setButtonHandler(enabled? this::driverControllerButtonEvent: null);
        }
        else
        {
            robot.leftDriveStick.setButtonHandler(enabled? this::leftDriveStickButtonEvent: null);
            robot.rightDriveStick.setButtonHandler(enabled? this::rightDriveStickButtonEvent: null);
        }
        robot.operatorStick.setButtonHandler(enabled? this::operatorStickButtonEvent: null);

        if (RobotParams.Preferences.useButtonPanels)
        {
            robot.buttonPanel.setButtonHandler(enabled? this::buttonPanelButtonEvent: null);
            robot.switchPanel.setButtonHandler(enabled? this::switchPanelButtonEvent: null);
        }
    }   //setControlsEnabled

    /**
     * This method returns robot heading to be maintained in teleop drive according to drive orientation mode.
     *
     * @return robot heading to be maintained.
     */
    private double getDriveGyroAngle()
    {
        switch (robot.robotDrive.driveOrientation)
        {
            case ROBOT:
                return 0.0;

            case INVERTED:
                return 180.0;

            default:
            case FIELD:
                return robot.robotDrive.driveBase.getHeading();
        }
    }   //getDriveGyroAngle

    //
    // Implements FrcButtonHandler.
    //

    /**
     * This method is called when a driver stick button event is detected.
     *
     * @param button specifies the button ID that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void driverControllerButtonEvent(int button, boolean pressed)
    {
        final String funcName = "driverControllerButtonEvent";

        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(funcName, ">>>>> button=%d, pressed=%s", button, pressed);
        }

        robot.dashboard.displayPrintf(
            8, "DriverController: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcXboxController.BUTTON_A:
                // Reset robot heading for field oriented driving.
                if (robot.robotDrive != null && pressed)
                {
                    TrcPose2D robotPose = robot.robotDrive.driveBase.getFieldPosition();
                    robotPose.angle = 0.0;
                    robot.robotDrive.driveBase.setFieldPosition(robotPose);
                }
                break;

            case FrcXboxController.BUTTON_B:
                // Evan's turtle mode, need to test ownership 
                if (pressed)
                {
                    robot.turtleMode(moduleName, null);
                }
                break;

            case FrcXboxController.BUTTON_X:
                break;

            case FrcXboxController.BUTTON_Y:
                // Toggle between Field or Robot oriented driving mode.
                if (pressed)
                {
                    if (robot.robotDrive.driveOrientation != RobotDrive.DriveOrientation.FIELD)
                    {
                        robot.robotDrive.setDriveOrientation(RobotDrive.DriveOrientation.FIELD);
                    }
                    else
                    {
                        robot.robotDrive.setDriveOrientation(RobotDrive.DriveOrientation.ROBOT);
                    }
                }
                break;

            case FrcXboxController.LEFT_BUMPER:
                // Press and hold for fast driving.
                if (pressed)
                {
                    robot.robotDrive.driveSpeedScale = RobotParams.DRIVE_FAST_SCALE;
                    // robot.robotDrive.turnSpeedScale = RobotParams.TURN_FAST_SCALE;
                }
                else
                {
                    robot.robotDrive.driveSpeedScale = RobotParams.DRIVE_MEDIUM_SCALE;
                    robot.robotDrive.turnSpeedScale = RobotParams.TURN_MEDIUM_SCALE;
                }
                break;

            case FrcXboxController.RIGHT_BUMPER:
                // Press and hold for slow driving.
                if (pressed)
                {
                    robot.robotDrive.driveSpeedScale = RobotParams.DRIVE_SLOW_SCALE;
                    robot.robotDrive.turnSpeedScale = RobotParams.TURN_SLOW_SCALE;
                }
                else
                {
                    robot.robotDrive.driveSpeedScale = RobotParams.DRIVE_MEDIUM_SCALE;
                    robot.robotDrive.turnSpeedScale = RobotParams.TURN_MEDIUM_SCALE;
                }
                break;

            case FrcXboxController.BACK:
                break;

            case FrcXboxController.START:
                break;

            case FrcXboxController.LEFT_STICK_BUTTON:
                break;

            case FrcXboxController.RIGHT_STICK_BUTTON:
                break;
        }
    }   //driverControllerButtonEvent

    /**
     * This method is called when a left driver stick button event is detected.
     *
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void leftDriveStickButtonEvent(int button, boolean pressed)
    {
        final String funcName = "leftDriveStickButtonEvent";

        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(funcName, ">>>>> button=%d, pressed=%s", button, pressed);
        }

        robot.dashboard.displayPrintf(
            8, "LeftDriveStick: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.LOGITECH_TRIGGER:
                break;

            case FrcJoystick.LOGITECH_BUTTON2:
                break;

            case FrcJoystick.LOGITECH_BUTTON3:
                break;

            case FrcJoystick.LOGITECH_BUTTON4:
                break;

            case FrcJoystick.LOGITECH_BUTTON5:
                break;

            case FrcJoystick.LOGITECH_BUTTON6:
                break;

            case FrcJoystick.LOGITECH_BUTTON7:
                break;

            case FrcJoystick.LOGITECH_BUTTON8:
                break;

            case FrcJoystick.LOGITECH_BUTTON9:
                break;

            case FrcJoystick.LOGITECH_BUTTON10:
                break;

            case FrcJoystick.LOGITECH_BUTTON11:
                break;

            case FrcJoystick.LOGITECH_BUTTON12:
                break;
        }
    }   //leftDriveStickButtonEvent

    /**
     * This method is called when a right driver stick button event is detected.
     *
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void rightDriveStickButtonEvent(int button, boolean pressed)
    {
        final String funcName = "rightDriveStickButtonEvent";

        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(funcName, ">>>>> button=%d, pressed=%s", button, pressed);
        }

        robot.dashboard.displayPrintf(
            8, "RightDriveStick: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.SIDEWINDER_TRIGGER:
                break;

            case FrcJoystick.LOGITECH_BUTTON3:
                break;

            case FrcJoystick.LOGITECH_BUTTON4:
                break;
        }
    }   //rightDriveStickButtonEvent

    /**
     * This method is called when an operator stick button event is detected.
     *
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void operatorStickButtonEvent(int button, boolean pressed)
    {
        final String funcName = "operatorStickButtonEvent";

        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(funcName, ">>>>> button=%d, pressed=%s", button, pressed);
        }

        robot.dashboard.displayPrintf(
            8, "OperatorStick: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.LOGITECH_TRIGGER:
                // Press to toggle between different modes of autoAssistIntake.
                if (robot.intake != null)
                {
                    if (pressed)
                    {
                        if (robot.intake.isAutoAssistActive())
                        {
                            robot.intake.autoAssistCancel();
                            robot.ledIndicator.setScoreLevel(robot.objType, robot.scoreLevel);
                        }
                        else if (!robot.intake.hasObject())
                        {
                            robot.intake.autoAssistIntake(
                                robot.objType == ObjectType.CONE?
                                    RobotParams.INTAKE_PICKUP_POWER: -RobotParams.INTAKE_PICKUP_POWER,
                                robot.objType == ObjectType.CONE? RobotParams.INTAKE_CONE_RETAIN_POWER: RobotParams.INTAKE_CUBE_RETAIN_POWER,
                                0.75);
                            robot.ledIndicator.setIntakeRunning(true, robot.objType, robot.scoreLevel);
                        }
                        else if (spitting)
                        {
                            robot.intake.setPower(robot.objType == ObjectType.CONE? RobotParams.INTAKE_CONE_SPIT_POWER: RobotParams.INTAKE_CUBE_SPIT_POWER);
                            // double finishDelay = robot.objType == ObjectType.CONE? 5.0 : 1.0;
                            // robot.intake.autoAssistSpitout(
                            //     robot.objType == ObjectType.CONE?
                            //         RobotParams.INTAKE_CONE_SPIT_POWER: RobotParams.INTAKE_CUBE_SPIT_POWER,
                            //     finishDelay);
                        }
                    }
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON2:
                // Press to set up for scoring on all levels.
                // TODO: Once presets are all determined, switch to AutoScore
                if (pressed)
                {
                    if (robot.objType == ObjectType.CONE)
                    {
                        robot.prepSubsystems(moduleName,
                            RobotParams.elevatorConeScorePresets[robot.scoreLevel],
                            RobotParams.armConeScorePresets[robot.scoreLevel],
                            RobotParams.wristConeScorePresets[robot.scoreLevel]);
                    }
                    else
                    {
                        robot.prepSubsystems(moduleName,
                            RobotParams.elevatorCubeScorePresets[robot.scoreLevel],
                            RobotParams.armCubeScorePresets[robot.scoreLevel],
                            RobotParams.wristCubeScorePresets[robot.scoreLevel]);
                    }
                    // robot.autoScoreTask.autoAssistScoreObject(robot.objType, robot.scoreLevel, null, false, true, null);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON3:
                spitting = pressed;
                // TODO (Code Review): This should use autoAssist!
                if (!pressed)
                {
                    robot.intake.setPower(0.0);
                }
                // if(!pressed && robot.autoScoreTask.isActive()) { robot.autoScoreTask.autoAssistCancel(); }
                break;
            
            //READY CUBE GROUND PICKUP
            case FrcJoystick.LOGITECH_BUTTON4:
                if (pressed)
                {
                    robot.prepForCubeGroundPickup(moduleName, 0.0, null);
                }
                break;
            
            //READY CONE GROUND PICKUP (NOSE IN)
            case FrcJoystick.LOGITECH_BUTTON5:
                if (pressed)
                {
                    robot.prepForConeGroundPickup(moduleName, 0.0, null);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON6:
                // Increases score level
                if (pressed && robot.scoreLevel < 2)
                {
                    robot.scoreLevel++;
                    robot.ledIndicator.setScoreLevel(robot.objType, robot.scoreLevel);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON7:
                // Lowers score level
                if (pressed && robot.scoreLevel > 0)
                {
                    robot.scoreLevel--;
                    robot.ledIndicator.setScoreLevel(robot.objType, robot.scoreLevel);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON8:
                break;

            case FrcJoystick.LOGITECH_BUTTON9:
                // Zero calibrate elevator.
                if (robot.elevator != null && pressed)
                {
                    robot.elevatorPidActuator.zeroCalibrate(moduleName);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON10:
                // Force wrist sync
                if (robot.wrist != null && pressed)
                {
                    robot.wrist.syncEncoder(true);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON11:
                break;

            case FrcJoystick.LOGITECH_BUTTON12:
                break;
        }
    }   //operatorStickButtonEvent

    /**
     * This method is called when a button panel button event is detected.
     *
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void buttonPanelButtonEvent(int button, boolean pressed)
    {
        final String funcName = "buttonPanelButtonEvent";

        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(funcName, ">>>>> button=%d, pressed=%s", button, pressed);
        }

        robot.dashboard.displayPrintf(
            8, "ButtonPanel: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            //ARM CONTROL
            case FrcJoystick.PANEL_BUTTON_RED1:
                // Press and hold to control the arm with operator joystick.
                armControl = pressed;
                if (armControl)
                {
                    // The operator stick is controlling the arm, disable wrist control.
                    wristControl = false;
                }
                else
                {
                    // Stop the arm on release but still holding its position.
                    robot.armPidActuator.setPidPower(0.0, true);
                }
                break;

            //WRIST CONTROL
            case FrcJoystick.PANEL_BUTTON_GREEN1:
                // Press and hold to control the wrist with operator joystick.
                wristControl = pressed;
                if (wristControl)
                {
                    // The operator stick is controlling the wrist, disable arm control.
                    armControl = false;
                }
                else
                {
                    // Stop the wrist on release but still holding its position.
                    robot.wristPidActuator.setPidPower(0.0, true);
                }
                break;

            case FrcJoystick.PANEL_BUTTON_BLUE1:
                manualOverride = pressed;
                break;

            //Prepare for Single Substation Pickup 
            case FrcJoystick.PANEL_BUTTON_YELLOW1:
                if (pressed)
                {
                    robot.prepForConeSubstationPickup(moduleName, 0.0, null);
                }
                break;
            
            //CANCEL BUTTON
            case FrcJoystick.PANEL_BUTTON_WHITE1:
                if (pressed)
                {
                    releaseAutoAssistAndSubsystems();
                }
                break;
            
            case FrcJoystick.PANEL_BUTTON_RED2:
                // TODO (Code Review): What is this? Why? You already have a pair of buttons doing level up and down!
                // If you prefer discrete buttons selecting score levels, then don't have up/down buttons but you are wasting buttons.
                if (pressed)
                {
                    robot.scoreLevel = 2;
                }
                break;
            
            case FrcJoystick.PANEL_BUTTON_GREEN2:
                // TODO (Code Review): Again, why?!
                if (pressed)
                {
                    robot.scoreLevel = 1;
                }
                break;

            //TURTLE MODE (Everything retracted)
            case FrcJoystick.PANEL_BUTTON_BLUE2:
                //evan also has turtle mode on Button B 
                if (pressed)
                {
                    robot.turtleMode(moduleName, null);
                }
                break;

            //CONE SHOOTING
            case FrcJoystick.PANEL_BUTTON_YELLOW2:
                // TODO (Code Review): Is this test code?
                if (pressed)
                {
                    //arm - 93.8, wrist - 199, elevator - 22.8
                    robot.prepSubsystems(moduleName,22.8, 93.8, 199);
                    // robot.intake.setPower(null, 0, -1.0, 1.0, null);
                }
                break;

            //Ready for Scoring low 
            case FrcJoystick.PANEL_BUTTON_WHITE2:
                break;
        }
    }   //buttonPanelButtonEvent

    /**
     * This method is called when a switch panel button event is detected.
     *
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void switchPanelButtonEvent(int button, boolean pressed)
    {
        final String funcName = "switchPanelButtonEvent";

        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(funcName, ">>>>> button=%d, pressed=%s", button, pressed);
        }

        robot.dashboard.displayPrintf(
            8, "SwitchPanel: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.PANEL_SWITCH_WHITE1:
                break;

            case FrcJoystick.PANEL_SWITCH_RED1:
                break;

            case FrcJoystick.PANEL_SWITCH_GREEN1:
                break;

            case FrcJoystick.PANEL_SWITCH_BLUE1:
                break;

            case FrcJoystick.PANEL_SWITCH_YELLOW1:
                break;

            case FrcJoystick.PANEL_SWITCH_WHITE2:
                break;

            case FrcJoystick.PANEL_SWITCH_RED2:
                break;

            case FrcJoystick.PANEL_SWITCH_GREEN2:
                break;

            case FrcJoystick.PANEL_SWITCH_BLUE2:
                break;

            case FrcJoystick.PANEL_SWITCH_YELLOW2:
                break;
        }
    }   //switchPanelButtonEvent

    private void releaseAutoAssistAndSubsystems()
    {
        if (robot.autoPickupTask != null) robot.autoPickupTask.autoAssistCancel();
        if (robot.autoScoreTask != null) robot.autoScoreTask.autoAssistCancel();
        if (robot.autoBalanceTask != null) robot.autoBalanceTask.autoAssistCancel();
        if (robot.elevator != null) robot.elevatorPidActuator.releaseExclusiveAccess(moduleName);
        if (robot.arm != null ) robot.armPidActuator.releaseExclusiveAccess(moduleName);
        if (robot.wrist != null) robot.wristPidActuator.releaseExclusiveAccess(moduleName);
        if (robot.intake != null) robot.intake.releaseExclusiveAccess(moduleName);
    }   //releaseAutoAssistAndSubsystems

}   //class FrcTeleOp
