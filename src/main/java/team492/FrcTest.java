/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
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

package team492;

import java.util.Locale;

import TrcCommonLib.command.CmdDriveMotorsTest;
import TrcCommonLib.command.CmdPidDrive;
import TrcCommonLib.command.CmdTimedDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.util.Color;
import TrcFrcLib.frclib.FrcChoiceMenu;
import TrcFrcLib.frclib.FrcUserChoices;
import TrcCommonLib.trclib.TrcColor;
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcRobot.RunMode;

/**
 * This class implements the code to run in Test Mode.
 */
public class FrcTest extends FrcTeleOp
{
    private static final String moduleName = "FrcTest";
    //
    // Global constants.
    //

    //
    // Tests.
    //
    private enum Test
    {
        SENSORS_TEST,
        SUBSYSTEMS_TEST,
        SWERVE_CALIBRATION,
        DRIVE_SPEED_TEST,
        DRIVE_MOTORS_TEST,
        X_TIMED_DRIVE,
        Y_TIMED_DRIVE,
        PP_DRIVE,
        PID_DRIVE,
        TUNE_X_PID,
        TUNE_Y_PID,
        TUNE_TURN_PID,
        LIVE_WINDOW
    }   //enum Test

    /**
     * This class encapsulates all user choices for test mode from the smart dashboard.
     *
     * To add a test choice, follow the steps below:
     * 1. Add a DBKEY string constant.
     * 2. If the choice is a choice menu, create a FrcChoiceMenu variable for it, create the enum type if necessary,
     *    add code to create the FrcChoiceMenu object and add choices to it.
     * 3. Call userChoices to add the new choice object and provide default value if necessary.
     * 4. Add a getter method for the new choice.
     * 5. Add an entry of the new choice to the toString method.
     */
    class TestChoices
    {
        // Smart dashboard keys for Autonomous choices.
        private static final String DBKEY_TEST_TESTS = "Test/Tests";
        private static final String DBKEY_TEST_X_DRIVE_DISTANCE = "Test/XDriveDistance";
        private static final String DBKEY_TEST_Y_DRIVE_DISTANCE = "Test/YDriveDistance";
        private static final String DBKEY_TEST_TURN_ANGLE = "Test/TurnAngle";
        private static final String DBKEY_TEST_DRIVE_TIME = "Test/DriveTime";
        private static final String DBKEY_TEST_DRIVE_POWER = "Test/DrivePower";
        private static final String DBKEY_TEST_TUNE_KP = "Test/TuneKp";
        private static final String DBKEY_TEST_TUNE_KI = "Test/TuneKi";
        private static final String DBKEY_TEST_TUNE_KD = "Test/TuneKd";
        private static final String DBKEY_TEST_TUNE_KF = "Test/TuneKf";
    
        private final FrcUserChoices userChoices = new FrcUserChoices();
        private final FrcChoiceMenu<Test> testMenu;

        public TestChoices()
        {
            //
            // Create test mode specific choice menus.
            //
            testMenu = new FrcChoiceMenu<>(DBKEY_TEST_TESTS);
            //
            // Populate test mode menus.
            //
            testMenu.addChoice("Sensors Test", Test.SENSORS_TEST, true, false);
            testMenu.addChoice("Subsystems Test", Test.SUBSYSTEMS_TEST);
            testMenu.addChoice("Swerve Calibration", Test.SWERVE_CALIBRATION);
            testMenu.addChoice("Drive Speed Test", Test.DRIVE_SPEED_TEST);
            testMenu.addChoice("Drive Motors Test", Test.DRIVE_MOTORS_TEST);
            testMenu.addChoice("X Timed Drive", Test.X_TIMED_DRIVE);
            testMenu.addChoice("Y Timed Drive", Test.Y_TIMED_DRIVE);
            testMenu.addChoice("PurePursuit Drive", Test.PP_DRIVE);
            testMenu.addChoice("PID Drive", Test.PID_DRIVE);
            testMenu.addChoice("Tune X PID", Test.TUNE_X_PID);
            testMenu.addChoice("Tune Y PID", Test.TUNE_Y_PID);
            testMenu.addChoice("Tune Turn PID", Test.TUNE_TURN_PID);
            testMenu.addChoice("Live Window", Test.LIVE_WINDOW, false, true);
            //
            // Initialize dashboard with default choice values.
            //
            userChoices.addChoiceMenu(DBKEY_TEST_TESTS, testMenu);
            userChoices.addNumber(DBKEY_TEST_X_DRIVE_DISTANCE, 6.0);    // in ft
            userChoices.addNumber(DBKEY_TEST_Y_DRIVE_DISTANCE, 6.0);    // in ft
            userChoices.addNumber(DBKEY_TEST_TURN_ANGLE, 90.0);         // in degrees
            userChoices.addNumber(DBKEY_TEST_DRIVE_TIME, 4.0);          // in seconds
            userChoices.addNumber(DBKEY_TEST_DRIVE_POWER, 0.5);
            userChoices.addNumber(DBKEY_TEST_TUNE_KP, 1.0);
            userChoices.addNumber(DBKEY_TEST_TUNE_KI, 0.0);
            userChoices.addNumber(DBKEY_TEST_TUNE_KD, 0.0);
            userChoices.addNumber(DBKEY_TEST_TUNE_KF, 0.0);
        }   //TestChoices

        //
        // Getters for test mode choices.
        //

        public Test getTest()
        {
            return testMenu.getCurrentChoiceObject();            
        }   //getTest

        public double getXDriveDistance()
        {
            return userChoices.getUserNumber(DBKEY_TEST_X_DRIVE_DISTANCE);
        }   //getXDriveDistance

        public double getYDriveDistance()
        {
            return userChoices.getUserNumber(DBKEY_TEST_Y_DRIVE_DISTANCE);
        }   //getYDriveDistance

        public double getTurnAngle()
        {
            return userChoices.getUserNumber(DBKEY_TEST_TURN_ANGLE);
        }   //getTurnAngle

        public double getDriveTime()
        {
            return userChoices.getUserNumber(DBKEY_TEST_DRIVE_TIME);
        }   //getDriveTime

        public double getDrivePower()
        {
            return userChoices.getUserNumber(DBKEY_TEST_DRIVE_POWER);
        }   //getDrivePower

        public TrcPidController.PidCoefficients getTunePidCoefficients()
        {
            return new TrcPidController.PidCoefficients(
                userChoices.getUserNumber(DBKEY_TEST_TUNE_KP),
                userChoices.getUserNumber(DBKEY_TEST_TUNE_KI),
                userChoices.getUserNumber(DBKEY_TEST_TUNE_KD),
                userChoices.getUserNumber(DBKEY_TEST_TUNE_KF));
        }   //getTunePidCoefficients

        @Override
        public String toString()
        {
            return String.format(
                Locale.US,
                "Test=\"%s\" " +
                "xDistance=%.1f ft " +
                "yDistance=%.1f ft " +
                "turnDegrees=%.0f deg " +
                "driveTime=%.0f sec " +
                "drivePower=%.1f " +
                "tunePidCoeff=%s ",
                getTest(), getXDriveDistance(), getYDriveDistance(), getTurnAngle(), getDriveTime(), getDrivePower(),
                getTunePidCoefficients());
        }   //toString

    }   //class TestChocies

    //
    // Global objects.
    //
    private final TestChoices testChoices = new TestChoices();
    private TrcRobot.RobotCommand testCommand;
    private double maxDriveVelocity = 0.0;
    private double maxDriveAcceleration = 0.0;
    private double maxTurnRate = 0.0;
    private double prevTime = 0.0;
    private double prevVelocity = 0.0;

    public FrcTest(Robot robot)
    {
        //
        // Call TeleOp constructor.
        //
        super(robot);
        //
        // Create and initialize global objects.
        //

    }   //FrcTest

    //
    // Overriding TrcRobot.RobotMode.
    //

    @Override
    public void startMode(RunMode prevMode, RunMode nextMode)
    {
        //
        // Call TeleOp startMode.
        //
        super.startMode(prevMode, nextMode);
        //
        // Retrieve Test choices.
        //
        robot.globalTracer.logInfo(moduleName, "TestChoices", "%s", testChoices);
        //
        // Create Command objects according to test choice.
        //
        boolean liveWindowEnabled = false;

        switch (testChoices.getTest())
        {
            case SENSORS_TEST:
                //
                // Make sure no joystick controls on sensors test.
                //
                setControlsEnabled(false);
                //
                // Sensors Test is the same as Subsystems Test without joystick control.
                // So let it flow to the next case.
                //
            case SUBSYSTEMS_TEST:
                break;

            case SWERVE_CALIBRATION:
                setControlsEnabled(false);
                robot.robotDrive.startSteerCalibrate();
                break;

            case DRIVE_MOTORS_TEST:
                //
                // Initialize motor array with the wheel motors. For 2-motor drive base, it is leftWheel and
                // rightWheel. For 4-motor drive base, it is lfWheel, rfWheel, lbWheel, rbWheel.
                //
                testCommand = new CmdDriveMotorsTest(
                    new TrcMotor[] {
                        robot.robotDrive.lfDriveMotor, robot.robotDrive.rfDriveMotor,
                        robot.robotDrive.lbDriveMotor, robot.robotDrive.rbDriveMotor},
                    5.0, 0.5);
                break;

            case X_TIMED_DRIVE:
                if (robot.robotDrive.driveBase.supportsHolonomicDrive())
                {
                    testCommand = new CmdTimedDrive(
                        robot.robotDrive.driveBase, 0.0, testChoices.getDriveTime(), testChoices.getDrivePower(),
                        0.0, 0.0);
                }
                break;

            case Y_TIMED_DRIVE:
                testCommand = new CmdTimedDrive(
                    robot.robotDrive.driveBase, 0.0, testChoices.getDriveTime(), 0.0, testChoices.getDrivePower(),
                    0.0);
                break;

            case PP_DRIVE:
                robot.robotDrive.purePursuitDrive.setMoveOutputLimit(testChoices.getDrivePower());
                robot.robotDrive.purePursuitDrive.start(
                    null, robot.robotDrive.driveBase.getFieldPosition(), true,
                    new TrcPose2D(
                        testChoices.getXDriveDistance()*12.0, testChoices.getYDriveDistance()*12.0,
                        testChoices.getTurnAngle()));
                break;

            case PID_DRIVE:
                testCommand = new CmdPidDrive(
                    robot.robotDrive.driveBase, robot.robotDrive.pidDrive, 0.0, testChoices.getDrivePower(), null,
                    new TrcPose2D(
                        testChoices.getXDriveDistance()*12.0, testChoices.getYDriveDistance()*12.0,
                        testChoices.getTurnAngle()));
                break;

            case TUNE_X_PID:
                if (robot.robotDrive.driveBase.supportsHolonomicDrive())
                {
                    testCommand = new CmdPidDrive(
                        robot.robotDrive.driveBase, robot.robotDrive.pidDrive, 0.0, testChoices.getDrivePower(),
                        testChoices.getTunePidCoefficients(), new TrcPose2D(testChoices.getXDriveDistance()*12.0,
                        0.0, 0.0));
                }
                break;

            case TUNE_Y_PID:
                testCommand = new CmdPidDrive(
                    robot.robotDrive.driveBase, robot.robotDrive.pidDrive, 0.0, testChoices.getDrivePower(),
                    testChoices.getTunePidCoefficients(), new TrcPose2D(0.0, testChoices.getYDriveDistance()*12.0,
                    0.0));
                break;

            case TUNE_TURN_PID:
                testCommand = new CmdPidDrive(
                    robot.robotDrive.driveBase, robot.robotDrive.pidDrive, 0.0, testChoices.getDrivePower(),
                    testChoices.getTunePidCoefficients(), new TrcPose2D(0.0, 0.0, testChoices.getTurnAngle()));
                break;

            case LIVE_WINDOW:
                liveWindowEnabled = true;
                break;

            default:
                break;
        }

        LiveWindow.setEnabled(liveWindowEnabled);
        //
        // Start test state machine if necessary.
        //

    }   //startMode

    @Override
    public void stopMode(RunMode prevMode, RunMode nextMode)
    {
        super.stopMode(prevMode, nextMode);
    }   //stopMode

    //
    // Must override TeleOp so it doesn't fight with us.
    //

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
        if (testCommand != null)
        {
            testCommand.cmdPeriodic(elapsedTime);
        }
        //
        // Run test Cmd.
        //
        switch (testChoices.getTest())
        {
            case SENSORS_TEST:
                super.periodic(elapsedTime, slowPeriodicLoop);
                break;

            case DRIVE_SPEED_TEST:
                double currTime = TrcTimer.getCurrentTime();
                TrcPose2D velPose = robot.robotDrive.driveBase.getFieldVelocity();
                double velocity = TrcUtil.magnitude(velPose.x, velPose.y);
                double acceleration = 0.0;
                double turnRate = robot.robotDrive.driveBase.getTurnRate();

                if (prevTime != 0.0)
                {
                    acceleration = (velocity - prevVelocity)/(currTime - prevTime);
                }

                if (velocity > maxDriveVelocity)
                {
                    maxDriveVelocity = velocity;
                }

                if (acceleration > maxDriveAcceleration)
                {
                    maxDriveAcceleration = acceleration;
                }

                if (turnRate > maxTurnRate)
                {
                    maxTurnRate = turnRate;
                }

                prevTime = currTime;
                prevVelocity = velocity;

                robot.dashboard.displayPrintf(9, "Drive Vel: (%.1f/%.1f)", velocity, maxDriveVelocity);
                robot.dashboard.displayPrintf(10, "Drive Accel: (%.1f/%.1f)", acceleration, maxDriveAcceleration);
                robot.dashboard.displayPrintf(11, "Turn Rate: (%.1f/%.1f)", turnRate, maxTurnRate);
                break;

            default:
                break;
        }

        if (slowPeriodicLoop)
        {
            if (allowTeleOp())
            {
                //
                // Allow TeleOp to run so we can control the robot in subsystem test or drive speed test modes.
                //
                super.periodic(elapsedTime, true);
            }

            //
            // Call super.runPeriodic only if you need TeleOp control of the robot.
            //
            switch (testChoices.getTest())
            {
                case SENSORS_TEST:
                case SUBSYSTEMS_TEST:
                    displaySensorStates();
                    break;

                case SWERVE_CALIBRATION:
                    robot.robotDrive.steerCalibratePeriodic();
                    displaySensorStates();
                    break;

                case X_TIMED_DRIVE:
                case Y_TIMED_DRIVE:
                    double lfEnc = robot.robotDrive.lfDriveMotor.getPosition();
                    double rfEnc = robot.robotDrive.rfDriveMotor.getPosition();
                    double lbEnc = robot.robotDrive.lbDriveMotor.getPosition();
                    double rbEnc = robot.robotDrive.rbDriveMotor.getPosition();
                    robot.dashboard.displayPrintf(9, "Enc:lf=%.0f,rf=%.0f", lfEnc, rfEnc);
                    robot.dashboard.displayPrintf(10, "Enc:lb=%.0f,rb=%.0f", lbEnc, rbEnc);
                    robot.dashboard.displayPrintf(11, "EncAverage=%f", (lfEnc + rfEnc + lbEnc + rbEnc) / 4.0);
                    robot.dashboard.displayPrintf(12, "RobotPose=%s", robot.robotDrive.driveBase.getFieldPosition());
                    break;

                case PP_DRIVE:
                case PID_DRIVE:
                case TUNE_X_PID:
                case TUNE_Y_PID:
                case TUNE_TURN_PID:
                    int lineNum = 10;
                    robot.dashboard.displayPrintf(9, "RobotPose=%s", robot.robotDrive.driveBase.getFieldPosition());
                    if (robot.robotDrive.encoderXPidCtrl != null)
                    {
                        robot.robotDrive.encoderXPidCtrl.displayPidInfo(lineNum);
                        lineNum += 2;
                    }
                    robot.robotDrive.encoderYPidCtrl.displayPidInfo(lineNum);
                    lineNum += 2;
                    robot.robotDrive.gyroTurnPidCtrl.displayPidInfo(lineNum);
                    break;

                default:
                    break;
            }
            //
            // Update Dashboard.
            //
            if (RobotParams.Preferences.doStatusUpdate)
            {
                robot.updateStatus();
            }
        }
    }   //periodic

    /**
     * This method is called to determine if Test mode is allowed to do teleop control of the robot.
     *
     * @return true to allow and false otherwise.
     */
    private boolean allowTeleOp()
    {
        Test test = testChoices.getTest();

        return test == Test.SUBSYSTEMS_TEST || test == Test.DRIVE_SPEED_TEST;
    }   //allowTeleOp

    //
    // Overriding ButtonEvent here if necessary.
    //

    //
    // Implement tests.
    //
    /**
     * This method reads all sensors and prints out their values. This is a very
     * useful diagnostic tool to check if all sensors are working properly. For
     * encoders, since test subsystem mode is also teleop mode, you can operate
     * the joysticks to turn the motors and check the corresponding encoder
     * counts.
     */
    private void displaySensorStates()
    {
        //
        // Display drivebase info.
        //
        if (robot.battery != null)
        {
            robot.dashboard.displayPrintf(
                9, "Sensors Test (Batt=%.1f/%.1f):", robot.battery.getVoltage(), robot.battery.getLowestVoltage());
        }
        robot.dashboard.displayPrintf(
            10, "DriveBase: Pose=%s,Vel=%s", robot.robotDrive.driveBase.getFieldPosition(),
            robot.robotDrive.driveBase.getFieldVelocity());
        robot.dashboard.displayPrintf(11, "DriveEncoders: lf=%.1f,rf=%.1f,lb=%.1f,rb=%.1f",
            robot.robotDrive.lfDriveMotor.getPosition(), robot.robotDrive.rfDriveMotor.getPosition(),
            robot.robotDrive.lbDriveMotor.getPosition(), robot.robotDrive.rbDriveMotor.getPosition());
        robot.dashboard.displayPrintf(12, "DrivePower: lf=%.2f,rf=%.2f,lb=%.2f,rb=%.2f",
            robot.robotDrive.lfDriveMotor.getMotorPower(), robot.robotDrive.rfDriveMotor.getMotorPower(),
            robot.robotDrive.lbDriveMotor.getMotorPower(), robot.robotDrive.rbDriveMotor.getMotorPower());
        Color color = robot.colorSensor.getColor();
        double[] hsv = TrcColor.rgbToHsv(color.red, color.green, color.blue);
        robot.dashboard.displayPrintf(
            13, "Color: RGB=(%.2f,%.2f,%.2f), HSV=(%.1f,%.1f,%.1f)",
            color.red, color.green, color.blue, hsv[0], hsv[1], hsv[2]);

        //
        // Display other subsystems and sensor info.
        //

    }   //displaySensorStates

}   //class FrcTest
