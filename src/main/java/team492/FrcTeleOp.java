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
import TrcCommonLib.trclib.TrcTriggerThresholdZones;
import TrcCommonLib.trclib.TrcRobot.RunMode;
import TrcFrcLib.frclib.FrcJoystick;
import TrcFrcLib.frclib.FrcXboxController;
import team492.FrcAuto.BalanceInitSide;
import team492.FrcAuto.ObjectType;
import team492.FrcAuto.ScoreLocation;
import team492.drivebases.RobotDrive;

/**
 * This class implements the code to run in TeleOp Mode.
 */
public class FrcTeleOp implements TrcRobot.RobotMode
{
    private static final String moduleName = "FrcTeleOp";
    //
    // Global objects.
    //
    protected final Robot robot;
    private final TrcTriggerThresholdZones elevatorTrigger;
    private boolean controlsEnabled = false;

    private boolean fastSpitOut = false; 
    private boolean intakeReversed = false;
    private boolean armControl = false;
    private boolean armPosControl = false;
    private boolean manualElevator = false;

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
        if (robot.elevator != null)
        {
            elevatorTrigger = new TrcTriggerThresholdZones(
                "elevatorTrigger", robot.elevator::getPosition, RobotParams.ELEVATOR_TRIGGERS, false);
        }
        else
        {
            elevatorTrigger = null;
        }
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
            robot.elevator.zeroCalibrate(moduleName);
        }

        if (elevatorTrigger != null)
        {
            elevatorTrigger.enableTrigger(this::elevatorTriggerCallback);
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
        if (elevatorTrigger != null)
        {
            elevatorTrigger.disableTrigger();
        }
    }   //stopMode

    /**
     * This method is called when the elevator position crosses a certain threshold.
     *
     * @param context specifies the callback parameters.
     */
    private void elevatorTriggerCallback(Object context)
    {
        TrcTriggerThresholdZones.CallbackContext params = (TrcTriggerThresholdZones.CallbackContext) context;

        if (robot.intake != null && params.prevZone == 0 && params.currZone == 1)
        {
            // Elevator is going up.
            robot.intake.retract();
        }
    }   //elevatorTriggerCallback

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

                    if (robot.robotDrive.driveBase.supportsHolonomicDrive())
                    {
                        robot.robotDrive.driveBase.holonomicDrive(
                            null, inputs[0], inputs[1], inputs[2], getDriveGyroAngle());
                    }
                    else
                    {
                        robot.robotDrive.driveBase.arcadeDrive(inputs[1], inputs[2]);
                    }
                }
                //
                // Analog control of subsystem is done here if necessary.
                //
                if (RobotParams.Preferences.useSubsystems)
                {
                    if (robot.elevator != null && !armControl)
                    {
                        double elevatorPower = robot.operatorStick.getYWithDeadband(true);
                        if (manualElevator)
                        {
                            robot.elevatorPidActuator.setPower(elevatorPower);
                        }
                        else
                        {
                            robot.elevatorPidActuator.setPidPower(elevatorPower, true);
                        }
                    }

                    if (robot.arm != null)
                    {
                        if (armControl)
                        {
                            double armPower = RobotParams.ARM_MAX_POWER * robot.operatorStick.getYWithDeadband(true);
                            robot.armPidActuator.setPidPower(armPower, true);
                        }
                        else if (armPosControl)
                        {
                            double armPos =
                                (1 - robot.operatorStick.getZ())/2.0 * RobotParams.ARM_SAFE_RANGE +
                                RobotParams.ARM_LOW_POS;
                            robot.armPidActuator.setPosition(armPos, true, RobotParams.ARM_MAX_POWER);
                            robot.dashboard.displayPrintf(1, "Arm: pos=%.2f", armPos);
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
                // Test auto balance from inside the community.
                if (robot.robotDrive != null && pressed)
                {
                    if (robot.autoBalanceTask.isActive())
                    {
                        robot.autoBalanceTask.autoAssistCancel();
                    }
                    else
                    {
                        robot.autoBalanceTask.autoAssistBalance(BalanceInitSide.INSIDE, null);
                    }
                }
                break;

            case FrcXboxController.START:
                // Test auto balance from outside the community.
                if (robot.robotDrive != null && pressed)
                {
                    if (robot.autoBalanceTask.isActive())
                    {
                        robot.autoBalanceTask.autoAssistCancel();
                    }
                    else
                    {
                        robot.autoBalanceTask.autoAssistBalance(BalanceInitSide.OUTSIDE, null);
                    }
                }
                break;

            case FrcXboxController.LEFT_STICK_BUTTON:
                break;

            case FrcXboxController.RIGHT_STICK_BUTTON:
                break;
        }
    }   //driverControllerButtonEvent

    /**
     * This method is called when a right driver stick button event is detected.
     *
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void leftDriveStickButtonEvent(int button, boolean pressed)
    {
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
        robot.dashboard.displayPrintf(
            8, "OperatorStick: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.LOGITECH_TRIGGER:
                // Press to set up intake to gather cone or cube, release to cancel.
                if (robot.intake != null)
                {
                    if (pressed)
                    {
                        double intakePower;

                        robot.intake.extend();
                        if (intakeReversed)
                        {
                            intakePower = fastSpitOut? -1.0: RobotParams.INTAKE_SPIT_POWER;
                        }
                        else
                        {
                            intakePower = RobotParams.INTAKE_CUBE_PICKUP_POWER;
                        }
                        robot.intake.setPower(intakePower);
                    }
                    else
                    {
                        robot.intake.cancel();
                    }
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON2:
                // Toggle intake extend and retract.
                if (robot.intake != null && pressed)
                {
                    if (robot.intake.isExtended())
                    {
                        robot.intake.retract();
                    }
                    else
                    {
                        robot.intake.extend();
                    }
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON3:
                // Press and hold to reverse spinning of intake.
                intakeReversed = pressed; 
                break;
            
            case FrcJoystick.LOGITECH_BUTTON4:
                // Toggle cube grabber (aka polycarb).
                if (robot.grabber != null && pressed)
                {
                    if (robot.grabber.grabbedCube())
                    {
                        robot.grabber.releaseCube();
                    }
                    else
                    {
                        robot.grabber.grabCube();
                    }
                }
                break;
            
            case FrcJoystick.LOGITECH_BUTTON5:
                // Toggle cone grabber (aka earmuffs).
                if (robot.grabber != null && pressed)
                {
                    if (robot.grabber.grabbedCone())
                    {
                        robot.grabber.releaseCone();
                    }
                    else
                    {
                        robot.grabber.grabCone();
                    }
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON6:
                // Raises the arm one preset position up.
                if (robot.arm != null && pressed)
                {
                    robot.armPidActuator.presetPositionUp(moduleName, RobotParams.ARM_MAX_POWER);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON7:
                // Lowers the arm one preset position down.
                if (robot.arm != null && pressed)
                {
                    robot.armPidActuator.presetPositionDown(moduleName, RobotParams.ARM_MAX_POWER);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON8:
                // Press to spit object out of intake, release to cancel.
                if (robot.intake != null)
                {
                    if (pressed)
                    {
                        robot.intake.setPower(RobotParams.INTAKE_SPIT_POWER);
                    }
                    else
                    {
                        robot.intake.cancel();
                    }
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON9:
                // Zero calibrate elevator.
                if (robot.elevator != null && pressed)
                {
                    robot.elevator.zeroCalibrate(moduleName);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON10:
                // Poke the cube to release it.
                if (robot.grabber != null && pressed)
                {
                    robot.grabber.extendPoker();
                    robot.grabber.retractPoker(0.5);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON11:
                // Raising the arm for scoring, retracting the intake too.
                if (robot.arm != null && robot.intake != null && pressed)
                {
                    robot.armPidActuator.setPosition(
                        moduleName, RobotParams.ARM_MAX_POS, true, RobotParams.ARM_MAX_POWER, null, 0.0);
                    robot.intake.retract(1.0);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON12:
                // Lowering the arm for traveling.
                if (robot.arm != null && pressed)
                {
                    robot.armPidActuator.setPosition(
                        moduleName, RobotParams.ARM_TRAVEL_POSITION, true, RobotParams.ARM_MAX_POWER, null, 0.0);
                }
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
        robot.dashboard.displayPrintf(
            8, "ButtonPanel: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.PANEL_BUTTON_RED1:
                // Press and hold to control the arm with operator joystick.
                armControl = pressed;
                if (robot.arm != null && !armControl)
                {
                    // On release, make sure we stop the arm.
                    robot.armPidActuator.setPidPower(0.0);
                }
                break;

            case FrcJoystick.PANEL_BUTTON_GREEN1:
                // Ready position for cones
                // Moves elevator up and moves arm to its lowest position
                // We grab cones with the earmuffs straight above them so when grabbing,
                // all we have to do is move the elevator down and close the earmuffs
                if (robot.elevator != null && robot.arm != null && pressed)
                {
                    robot.elevatorPidActuator.setPosition(
                        moduleName, 0.0, 10.0, true, 1.0, null, 1.5);
                    robot.armPidActuator.setPosition(
                        moduleName, 0.0, RobotParams.ARM_MIN_POS_INTAKE_DOWN, true, RobotParams.ARM_MAX_POWER, null, 1.5);
                    robot.intake.extend();
                }
                break;

            case FrcJoystick.PANEL_BUTTON_BLUE1:
                // Cube pickup
                // Opens both grabbers
                // Moves arm down (elevator is already all the way down)
                // Closes cube grabber after a short delay
                if (robot.elevator != null && robot.arm != null && robot.grabber != null && pressed)
                {
                    robot.grabber.releaseCube();
                    robot.grabber.releaseCone();
                    robot.armPidActuator.setPosition(
                        moduleName, 0.3, RobotParams.ARM_MIN_POS_INTAKE_DOWN, true, RobotParams.ARM_MAX_POWER, null, 0.8);
                    robot.grabber.grabCube(0.8);
                }
                break;

            case FrcJoystick.PANEL_BUTTON_YELLOW1:
                // Cone pickup
                if (robot.elevator != null && robot.arm != null && robot.grabber != null && pressed)
                {
                    //AutoPickup Vision Version
                    // pickupObject = ObjectType.CONE;
                    // robot.photonVision.setPipeline(PipelineType.CONE);
                    //robot.autoPickupTask.autoAssistPickup(ObjectType.CONE, false, true, null);

                    //AutoPickup PickupOnly
                     robot.autoPickupTask.autoAssistPickup(ObjectType.CONE, false, true, null);

                    //AutoPickup Manual Version
                    // robot.grabber.grabCube();
                    // robot.grabber.releaseCone();
                    // robot.elevatorPidActuator.setPosition(
                    //     moduleName, 0.2, RobotParams.ELEVATOR_MIN_POS, true, 1.0, null, 1.5);
                    // robot.armPidActuator.setPosition(
                    //     moduleName, 0.0, RobotParams.ARM_MIN_POS, true, RobotParams.ARM_MAX_POWER, null, 1.5);
                    // //not sure if this works
                    // robot.grabber.grabCone(1.5);
                }
                break;

            case FrcJoystick.PANEL_BUTTON_WHITE1:
                if (pressed)
                {
                    if (robot.autoPickupTask != null) robot.autoPickupTask.autoAssistCancel();
                    if (robot.autoScoreTask != null) robot.autoScoreTask.autoAssistCancel();
                    if (robot.arm != null ) robot.armPidActuator.releaseExclusiveAccess(moduleName);
                    if (robot.elevator != null) robot.elevatorPidActuator.releaseExclusiveAccess(moduleName);
                }
                break;

            case FrcJoystick.PANEL_BUTTON_RED2:
                // manualElevator = pressed;
                //auto score testing
                if (robot.autoScoreTask != null && pressed)
                {
                    robot.autoScoreTask.autoAssistScoreObject(ObjectType.CONE, 2, ScoreLocation.LEFT, true, null);
                }
                break;

            case FrcJoystick.PANEL_BUTTON_GREEN2:
                // Ready for cube
                if (robot.elevator != null && robot.arm != null && pressed)
                {
                    robot.elevatorPidActuator.setPosition(
                        moduleName, 0.0, 0.0, true, 1.0, null, 1.5);
                    robot.armPidActuator.setPosition(
                        moduleName, 0.0, 60.0, true, RobotParams.ARM_MAX_POWER, null, 1.5);
                    robot.intake.extend();
                }
                break;

            case FrcJoystick.PANEL_BUTTON_BLUE2:
                break;
                // TURTLE MODE: TODO: Add label on button panel
                //use this for defense/before balance
                //assumes that when the intake goes up it will not hit the arm (do arm up beforehand)

            case FrcJoystick.PANEL_BUTTON_YELLOW2:
                //nose out cone pickup
                if (robot.elevator != null && robot.arm != null && robot.intake != null && pressed)
                {
                    robot.elevatorPidActuator.setPosition(moduleName, 0.0, true, 1.0, null, 0.0);
                    robot.armPidActuator.setPosition(
                        moduleName, 1.0, RobotParams.ARM_MIN_POS_INTAKE_UP, true, RobotParams.ARM_MAX_POWER, null,
                        0.0);
                    robot.intake.retract();
                }
                break;

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

}   //class FrcTeleOp
