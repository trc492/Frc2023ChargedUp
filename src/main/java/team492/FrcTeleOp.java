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
import team492.FrcAuto.ScoreLocation;
import team492.FrcAuto.BalanceStrafeDir;
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
    private boolean controlsEnabled = false;

    private boolean fastIntake = false;
    private boolean intakeReversed = false;

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
        //
        // Do subsystem auto-assist here if necessary.
        //

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
                    if (robot.elevator != null)
                    {
                        double elevatorPower = 0.0;
                        if (RobotParams.Preferences.useOperatorXboxController)
                        {
                            elevatorPower = robot.operatorController.getRightYWithDeadband(true);
                        }
                        else
                        {
                            elevatorPower = robot.operatorStick.getYWithDeadband(true);
                        }
                        // robot.elevatorPidActuator.setPower(elevatorPower);
                        robot.elevatorPidActuator.setPidPower(elevatorPower, true);
                    }

                    if (robot.arm != null)
                    {
                        if(RobotParams.Preferences.useOperatorXboxController)
                        {
                            double armPower = robot.operatorController.getLeftYWithDeadband(true);
                            robot.armPidActuator.setPidPower(armPower, true);
                        }
                        else
                        {
                            // double armPos =
                            //     (1 - robot.operatorStick.getZ())/2.0 * RobotParams.ARM_SAFE_RANGE +
                            //     RobotParams.ARM_LOW_POS;
                            // robot.armPidActuator.setPosition(armPos, true, RobotParams.ARM_MAX_POWER);
                        }
                    }

                    // if (robot.intake != null)
                    // {
                    //     double intakeLeftPower = robot.operatorController.getLeftTriggerWithDeadband(true);
                    //     double intakeRightPower = robot.operatorController.getRightTriggerWithDeadband(true);
                    //     if (intakeRightPower > 0.0)
                    //     {
                    //         robot.intake.setPower(RobotParams.INTAKE_CONE_PICKUP_POWER, RobotParams.INTAKE_CONE_PICKUP_POWER);
                    //     }
                    //     if (intakeLeftPower > 0.0)
                    //     {
                    //         robot.intake.setPower(RobotParams.INTAKE_CUBE_PICKUP_POWER, RobotParams.INTAKE_CUBE_PICKUP_POWER);
                    //     }
                    // }
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

        if (RobotParams.Preferences.useOperatorXboxController)
        {
            robot.operatorController.setButtonHandler(enabled? this::operatorControllerButtonEvent: null);
        }
        else
        {
            robot.operatorStick.setButtonHandler(enabled? this::operatorStickButtonEvent: null);
        }

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
    private void driverControllerButtonEvent(int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(
            8, "DriverController: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcXboxController.BUTTON_A:
                // Reset robot heading.
                if (pressed)
                {
                    TrcPose2D robotPose = robot.robotDrive.driveBase.getFieldPosition();
                    robotPose.angle = 0.0;
                    robot.robotDrive.driveBase.setFieldPosition(robotPose);
                }
                break;

            case FrcXboxController.BUTTON_B:
                break;

            case FrcXboxController.BUTTON_X:
                robot.robotDrive.setAntiDefenseEnabled(moduleName, pressed);
                break;

            case FrcXboxController.BUTTON_Y:
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
                if (pressed)
                {
                    // Reset all swerve steering to point absolute forward.
                    robot.robotDrive.setSteerAngleZero(false);
                }
                break;

            case FrcXboxController.RIGHT_BUMPER:
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
                if (pressed)
                {
                    if (robot.autoBalanceTask.isActive())
                    {
                        robot.autoBalanceTask.autoAssistCancel();
                    }
                    else
                    {
                        robot.autoBalanceTask.autoAssistBalance(BalanceStrafeDir.LEFT, null);
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
    private void leftDriveStickButtonEvent(int button, boolean pressed)
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
    private void rightDriveStickButtonEvent(int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(
            8, "RightDriveStick: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.SIDEWINDER_TRIGGER:
                if (pressed)
                {
                    robot.robotDrive.setDriveOrientation(RobotDrive.DriveOrientation.INVERTED);
                }
                else
                {
                    robot.robotDrive.setDriveOrientation(RobotDrive.DriveOrientation.FIELD);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON3:
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

            case FrcJoystick.LOGITECH_BUTTON4:
                if (pressed)
                {
                    TrcPose2D robotPose = robot.robotDrive.driveBase.getFieldPosition();
                    robotPose.angle = 0.0;
                    robot.robotDrive.driveBase.setFieldPosition(robotPose);
                }
                break;
        }
    }   //rightDriveStickButtonEvent

    /**
     * This method is called when an operator stick button event is detected.
     *
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    private void operatorStickButtonEvent(int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(
            8, "OperatorStick: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            //gathers cones, cubes into the intake 
            //if pressed, extends intake if its not extended, spins intake
            //otherwise stops
            case FrcJoystick.LOGITECH_TRIGGER:
                if (pressed)
                {
                    robot.intake.extend();
                    double intakePower = fastIntake? RobotParams.INTAKE_CONE_PICKUP_POWER: RobotParams.INTAKE_CUBE_PICKUP_POWER;
                    if(intakeReversed){
                        robot.intake.setPower(RobotParams.INTAKE_SPIT_POWER, RobotParams.INTAKE_SPIT_POWER);
                    }
                    else{
                        robot.intake.setPower(intakePower);
                    }
                }
                else
                {
                    robot.intake.cancel();
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON2:
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
                intakeReversed = pressed; 
                break;
            
            case FrcJoystick.LOGITECH_BUTTON4:
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
            //raises the arm and elevator one preset position
            case FrcJoystick.LOGITECH_BUTTON6:
                if (robot.arm != null && pressed)
                {
                    robot.armPidActuator.presetPositionUp(moduleName, RobotParams.ARM_MAX_POWER);
                }
                break;
            //lowers the arm and elevator one preset position 
            case FrcJoystick.LOGITECH_BUTTON7:
                if (robot.arm != null && pressed)
                {
                    robot.armPidActuator.presetPositionDown(moduleName, RobotParams.ARM_MAX_POWER);
                }
                break;

            //spits objects out 
            case FrcJoystick.LOGITECH_BUTTON8:
                if (pressed)
                {
                    robot.intake.setPower(RobotParams.INTAKE_SPIT_POWER);
                }
                else
                {
                    robot.intake.cancel();
                }
                // if(pressed){
                //     robot.intake.acquireExclusiveAccess("teleop");
                //     robot.intake.setPower("teleop", 0.0, -RobotParams.INTAKE_LEFT_PICKUP_POWER, -RobotParams.INTAKE_RIGHT_PICKUP_POWER, 0.0);
                // }
                // else{
                //     robot.intake.releaseExclusiveAccess("teleop");
                //     robot.intake.cancel("teleop");
                // }
                break;

            case FrcJoystick.LOGITECH_BUTTON9:
                if (robot.elevator != null && pressed)
                {
                    robot.elevatorPidActuator.zeroCalibrate(moduleName);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON10:
                if (pressed)
                {
                    robot.armPidActuator.setPosition(moduleName, 0.0, RobotParams.ARM_MAX_POS, true, RobotParams.ARM_MAX_POWER, null, 0.0);
                    // robot.autoPickupTask.autoAssistPickup(ObjectType.CONE, false, null);
                }
                else
                {
                    robot.armPidActuator.setPosition(moduleName,0.0,RobotParams.ARM_LOW_POS,true,RobotParams.ARM_MAX_POWER,null,0.0);
                }
                // if (robot.elevator != null && pressed)
                // {
                //     // Must acquire ownership to override analog control of the elevator.
                //     robot.elevatorPidActuator.presetPositionDown(moduleName);
                // }
                break;

            case FrcJoystick.LOGITECH_BUTTON11:
                // if (robot.elevator != null && pressed)
                // {
                //     // Must acquire ownership to override analog control of the elevator.
                //     robot.elevatorPidActuator.presetPositionUp(moduleName);
                // }
                //puts robot in scoring position for high poles 
                // robot.elevatorPidActuator.setPosition(moduleName, RobotParams.ELEVATOR_MAX_POS, true, 1.0, null, 0.0); 
                robot.armPidActuator.setPosition(moduleName, RobotParams.ARM_MAX_POS, true, 1.0, null, 0.0); 
                robot.intake.retract(1.0); 
                break;

            case FrcJoystick.LOGITECH_BUTTON12:
                // Turtle mode, use this while driving.
                // Must acquire ownership moving the elevator and arm or analog control will interfere.
                // robot.elevatorPidActuator.setPosition(moduleName, 0.0, true, 1.0, null, 0.0);
                robot.armPidActuator.setPosition(moduleName, RobotParams.ARM_TRAVEL_POSITION, true, RobotParams.ARM_MAX_POWER, null, 0.0);
                break;
        }
    }   //operatorStickButtonEvent

    /**
     * This method is called when an operator stick button event is detected.
     *
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    private void operatorControllerButtonEvent(int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(
            8, "OperatorController: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcXboxController.BUTTON_A:
                break;

            case FrcXboxController.BUTTON_B:
                break;

            case FrcXboxController.BUTTON_X:
                if (robot.grabber != null && pressed)
                {
                    if(robot.grabber.grabbedCube())
                    {
                        robot.grabber.releaseCube();
                    }
                    else
                    {
                        robot.grabber.grabCube();
                    }
                }
                break;

            case FrcXboxController.BUTTON_Y:
                if (robot.grabber != null && pressed)
                {
                    if(robot.grabber.grabbedCone())
                    {
                        robot.grabber.releaseCone();
                    }
                    else
                    {
                        robot.grabber.grabCone();
                    }
                }
                break;

            case FrcXboxController.LEFT_BUMPER:
                intakeReversed = pressed;
                break;

            case FrcXboxController.RIGHT_BUMPER:
                if (robot.intake != null && pressed)
                {
                    if(robot.intake.isExtended())
                    {
                        robot.intake.retract();
                    }
                    else
                    {
                        robot.intake.extend();
                    }
                }
                break;

            case FrcXboxController.BACK:
                // if (pressed)
                // {
                //     // TODO (Code Review): Why not use presetPositionDown???
                //     if(elevatorPresetIndex >= 0)
                //     {
                //         elevatorPresetIndex--;
                //     }
                //     robot.elevatorPidActuator.setPosition(RobotParams.elevatorPresets[elevatorPresetIndex], true);
                // }
                break;

            case FrcXboxController.START:
                break;

            case FrcXboxController.LEFT_STICK_BUTTON:
                break;

            case FrcXboxController.RIGHT_STICK_BUTTON:
                // if (pressed)
                // {
                //     // TODO (Code Review): Why not use presetPositionUp???
                //     if(elevatorPresetIndex < 6)
                //     {
                //         elevatorPresetIndex++;
                //     }
                //     robot.elevatorPidActuator.setPosition(RobotParams.elevatorPresets[elevatorPresetIndex], true);
                // }
                break; 
        }  
    }

    /**
     * This method is called when a button panel button event is detected.
     *
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    private void buttonPanelButtonEvent(int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(
            8, "ButtonPanel: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.PANEL_BUTTON_RED1:
                if(pressed){
                    robot.autoScoreTask.autoAssistScoreObject(ObjectType.CUBE, 2, ScoreLocation.MIDDLE, false, null);
                }
                // if (robot.arm != null && !pressed)
                // {
                //     robot.armPidActuator.setPower(0.0);
                // }
                //prepare for pickup
                //extend intake
                //lower the arm and elevator to min position
                break;

            case FrcJoystick.PANEL_BUTTON_GREEN1:
                    //reversing intake 
                intakeReversed = pressed; 
                break;

            case FrcJoystick.PANEL_BUTTON_BLUE1:
                break;

            case FrcJoystick.PANEL_BUTTON_YELLOW1:
                break;

            case FrcJoystick.PANEL_BUTTON_WHITE1:
                if (pressed)
                {
                    robot.autoPickupTask.autoAssistCancel();
                }
                break;

            case FrcJoystick.PANEL_BUTTON_RED2:
                break;

            case FrcJoystick.PANEL_BUTTON_GREEN2:
                break;

            case FrcJoystick.PANEL_BUTTON_BLUE2:
                break;

            case FrcJoystick.PANEL_BUTTON_YELLOW2:
                break;

            case FrcJoystick.PANEL_BUTTON_WHITE2:
                // TODO (Code Review): This code won't work because it doesn't release ownership. See operationStick button 9.
                // if (robot.elevator != null && pressed && robot.elevatorPidActuator.acquireExclusiveAccess("TeleOp"))
                // {
                //     if(robot.elevatorPidActuator.acquireExclusiveAccess("TeleOp"))
                //     {
                //         robot.elevatorPidActuator.zeroCalibrate("TeleOp");
                //     }
                //     else
                //     {
                //         robot.dashboard.displayPrintf(
                //             2, "elevator could not get owner");
                //     }
                // }
                break;
        }
    }   //buttonPanelButtonEvent

    /**
     * This method is called when a switch panel button event is detected.
     *
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    private void switchPanelButtonEvent(int button, boolean pressed)
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
