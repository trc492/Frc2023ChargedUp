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

package team492;

import java.util.Locale;

import TrcCommonLib.command.CmdDriveMotorsTest;
import TrcCommonLib.command.CmdPidDrive;
import TrcCommonLib.command.CmdTimedDrive;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import team492.FrcAuto.BalanceInitSide;
import team492.drivebases.SwerveDrive;
import team492.vision.OpenCvVision.ObjectType;
import team492.vision.PhotonVision.PipelineType;
import TrcFrcLib.frclib.FrcAnalogEncoder;
import TrcFrcLib.frclib.FrcChoiceMenu;
import TrcFrcLib.frclib.FrcPhotonVision;
import TrcFrcLib.frclib.FrcUserChoices;
import TrcFrcLib.frclib.FrcXboxController;
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcOpenCvDetector;
import TrcCommonLib.trclib.TrcPidActuator;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcVisionTargetInfo;
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
        VISION_TEST,
        SWERVE_CALIBRATION,
        DEBUG_SWERVE_STEERING,
        DRIVE_SPEED_TEST,
        DRIVE_MOTORS_TEST,
        TUNE_ARM_PID,
        TUNE_ELEVATOR_PID,
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
            testMenu.addChoice("Vision Test", Test.VISION_TEST);
            testMenu.addChoice("Swerve Calibration", Test.SWERVE_CALIBRATION);
            testMenu.addChoice("Debug Swerve Steering", Test.DEBUG_SWERVE_STEERING);
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
            userChoices.addNumber(DBKEY_TEST_X_DRIVE_DISTANCE, 0.0);    // in ft
            userChoices.addNumber(DBKEY_TEST_Y_DRIVE_DISTANCE, 0.0);    // in ft
            userChoices.addNumber(DBKEY_TEST_TURN_ANGLE, 0.0);         // in degrees
            userChoices.addNumber(DBKEY_TEST_DRIVE_TIME, 0.0);          // in seconds
            userChoices.addNumber(DBKEY_TEST_DRIVE_POWER, 0.5);
            userChoices.addNumber(DBKEY_TEST_TUNE_KP, RobotParams.SWERVE_KP);
            userChoices.addNumber(DBKEY_TEST_TUNE_KI, RobotParams.SWERVE_KI);
            userChoices.addNumber(DBKEY_TEST_TUNE_KD, RobotParams.SWERVE_KD);
            userChoices.addNumber(DBKEY_TEST_TUNE_KF, RobotParams.SWERVE_KF);
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
    private double[] steerZeros = new double[4];
    private long steerZeroSumCount = 0;
    private boolean swerveCalibrating = false;
    private boolean testSwerveSteering = false;
    private double swerveSteerAngle = 0.0;
    private double nextSteerAngleTime = 0.0;

    private TrcPidController tunePidCtrl = null;
    private TrcPidController.PidCoefficients savedPidCoeffs = null;

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

            case VISION_TEST:
                if (robot.openCvVision != null)
                {
                    robot.openCvVision.setDetectObjectType(ObjectType.APRILTAG);
                    robot.openCvVision.setVideoOutput(0, true);
                }
                break;

            case DRIVE_SPEED_TEST:
                // Overriding FrcTeleOp start mode init so we are really testing robot's top speed and acceleration.
                robot.robotDrive.driveSpeedScale = RobotParams.DRIVE_FAST_SCALE;
                robot.robotDrive.turnSpeedScale = RobotParams.TURN_FAST_SCALE;
                break;

            case DRIVE_MOTORS_TEST:
                // Initialize motor array with the wheel motors. For 2-motor drive base, it is leftWheel and
                // rightWheel. For 4-motor drive base, it is lfWheel, rfWheel, lbWheel, rbWheel.
                testCommand = new CmdDriveMotorsTest(
                    new TrcMotor[] {
                        robot.robotDrive.lfDriveMotor, robot.robotDrive.rfDriveMotor,
                        robot.robotDrive.lbDriveMotor, robot.robotDrive.rbDriveMotor},
                    5.0, 0.5);
                break;

            case TUNE_ARM_PID:
                if (robot.arm != null)
                {
                    tunePidActuator(robot.armPidActuator);
                }
                break;

            case TUNE_ELEVATOR_PID:
                if (robot.elevator != null)
                {
                    tunePidActuator(robot.elevatorPidActuator);
                }
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

                robot.dashboard.displayPrintf(1, "Drive Vel: (%.1f/%.1f)", velocity, maxDriveVelocity);
                robot.dashboard.displayPrintf(2, "Drive Accel: (%.1f/%.1f)", acceleration, maxDriveAcceleration);
                robot.dashboard.displayPrintf(3, "Turn Rate: (%.1f/%.1f)", turnRate, maxTurnRate);
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

                case VISION_TEST:
                    doVisionTest();
                    break;

                case SWERVE_CALIBRATION:
                    if (robot.robotDrive != null && robot.robotDrive instanceof SwerveDrive && swerveCalibrating)
                    {
                        SwerveDrive swerveDrive = (SwerveDrive) robot.robotDrive;
                        double[] rawPos = new double[steerZeros.length];

                        for (int i = 0; i < steerZeros.length; i++)
                        {
                            rawPos[i] = swerveDrive.steerEncoders[i].getRawPosition();
                            steerZeros[i] += rawPos[i];
                        }
                        steerZeroSumCount++;
                        robot.dashboard.displayPrintf(
                            1, "SteerEncVolt(%.3f): lf:%.3f, rf:%.3f, lb:%.3f, rb:%.3f",
                            RobotController.getVoltage5V(),
                            ((FrcAnalogEncoder) swerveDrive.steerEncoders[0]).getRawVoltage(),
                            ((FrcAnalogEncoder) swerveDrive.steerEncoders[1]).getRawVoltage(),
                            ((FrcAnalogEncoder) swerveDrive.steerEncoders[2]).getRawVoltage(),
                            ((FrcAnalogEncoder) swerveDrive.steerEncoders[3]).getRawVoltage());
                        robot.dashboard.displayPrintf(
                            2, "SteerEnc(Raw/Avg): lf=%.3f/%f, rf=%.3f/%f, lb=%.3f/%f, rb=%.3f/%f",
                            rawPos[0], steerZeros[0]/steerZeroSumCount, rawPos[1], steerZeros[1]/steerZeroSumCount,
                            rawPos[2], steerZeros[2]/steerZeroSumCount, rawPos[3], steerZeros[3]/steerZeroSumCount);
                        robot.dashboard.displayPrintf(
                            3, "SteerEncPos: lf=%.3f, rf=%.3f, lb=%.3f, rb=%.3f",
                            swerveDrive.steerEncoders[0].getPosition(), swerveDrive.steerEncoders[1].getPosition(),
                            swerveDrive.steerEncoders[2].getPosition(), swerveDrive.steerEncoders[3].getPosition());
                    }
                    break;

                case DEBUG_SWERVE_STEERING:
                    if (robot.robotDrive != null && robot.robotDrive instanceof SwerveDrive)
                    {
                        double currTime = TrcTimer.getCurrentTime();
                        if (testSwerveSteering && currTime > nextSteerAngleTime)
                        {
                            robot.globalTracer.traceInfo(
                                "DebugSwerveSteering", "Angle=%3.0f, lfVolt=%.3f, rfVolt=%.3f, lbVolt=%.3f, rbVolt=%.3f",
                                swerveSteerAngle,
                                ((FrcAnalogEncoder) robot.robotDrive.lfSteerEncoder).getRawVoltage(),
                                ((FrcAnalogEncoder) robot.robotDrive.rfSteerEncoder).getRawVoltage(),
                                ((FrcAnalogEncoder) robot.robotDrive.lbSteerEncoder).getRawVoltage(),
                                ((FrcAnalogEncoder) robot.robotDrive.rbSteerEncoder).getRawVoltage());
                            robot.robotDrive.lfWheel.setSteerAngle(swerveSteerAngle, false);
                            robot.robotDrive.rfWheel.setSteerAngle(swerveSteerAngle, false);
                            robot.robotDrive.lbWheel.setSteerAngle(swerveSteerAngle, false);
                            robot.robotDrive.rbWheel.setSteerAngle(swerveSteerAngle, false);
                            swerveSteerAngle++;
                            if (swerveSteerAngle >= 360.0)
                            {
                                testSwerveSteering = false;
                            }
                            nextSteerAngleTime = currTime + 0.1;
                        }
                        robot.robotDrive.displaySteerEncoders(1);
                    }
                    break;

                case X_TIMED_DRIVE:
                case Y_TIMED_DRIVE:
                    double lfEnc = robot.robotDrive.lfDriveMotor.getPosition();
                    double rfEnc = robot.robotDrive.rfDriveMotor.getPosition();
                    double lbEnc = robot.robotDrive.lbDriveMotor.getPosition();
                    double rbEnc = robot.robotDrive.rbDriveMotor.getPosition();
                    robot.dashboard.displayPrintf(1, "Enc:lf=%.0f,rf=%.0f", lfEnc, rfEnc);
                    robot.dashboard.displayPrintf(2, "Enc:lb=%.0f,rb=%.0f", lbEnc, rbEnc);
                    robot.dashboard.displayPrintf(3, "EncAverage=%f", (lfEnc + rfEnc + lbEnc + rbEnc) / 4.0);
                    robot.dashboard.displayPrintf(4, "RobotPose=%s", robot.robotDrive.driveBase.getFieldPosition());
                    break;

                case PP_DRIVE:
                case PID_DRIVE:
                case TUNE_X_PID:
                case TUNE_Y_PID:
                case TUNE_TURN_PID:
                    TrcPidController xPidCtrl, yPidCtrl, turnPidCtrl;
                    int lineNum = 2;

                    if (testChoices.getTest() == Test.PP_DRIVE)
                    {
                        xPidCtrl = robot.robotDrive.purePursuitDrive.getXPosPidCtrl();
                        yPidCtrl = robot.robotDrive.purePursuitDrive.getYPosPidCtrl();
                        turnPidCtrl = robot.robotDrive.purePursuitDrive.getTurnPidCtrl();
                    }
                    else
                    {
                        xPidCtrl = robot.robotDrive.pidDrive.getXPidCtrl();
                        yPidCtrl = robot.robotDrive.pidDrive.getYPidCtrl();
                        turnPidCtrl = robot.robotDrive.pidDrive.getTurnPidCtrl();
                    }

                    robot.dashboard.displayPrintf(1, "RobotPose=%s", robot.robotDrive.driveBase.getFieldPosition());
                    if (xPidCtrl != null)
                    {
                        xPidCtrl.displayPidInfo(lineNum);
                        lineNum += 2;
                    }
                    yPidCtrl.displayPidInfo(lineNum);
                    lineNum += 2;
                    turnPidCtrl.displayPidInfo(lineNum);
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

        return test == Test.SUBSYSTEMS_TEST || test == Test.VISION_TEST || test == Test.DRIVE_SPEED_TEST ||
               test == Test.SWERVE_CALIBRATION;
    }   //allowTeleOp

    //
    // Overrides ButtonHandler in TeleOp.
    //

    /**
     * This method is called when an operator stick button event is detected.
     *
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    @Override
    protected void driverControllerButtonEvent(int button, boolean pressed)
    {
        if (allowTeleOp())
        {
            boolean processed = false;
            //
            // In addition to or instead of the button controls handled by FrcTeleOp, we can add to or override the
            // FrcTeleOp button actions.
            //
            robot.dashboard.displayPrintf(
                8, "DriverController: button=0x%04x %s", button, pressed ? "pressed" : "released");
            switch (button)
            {
                case FrcXboxController.BUTTON_A:
                    if (testChoices.getTest() == Test.SWERVE_CALIBRATION &&
                        robot.robotDrive != null && robot.robotDrive instanceof SwerveDrive)
                    {
                        if (pressed)
                        {
                            startSwerveCalibration();
                            swerveCalibrating = true;
                        }
                        processed = true;
                    }
                    break;

                case FrcXboxController.BUTTON_B:
                    if (testChoices.getTest() == Test.SWERVE_CALIBRATION &&
                        robot.robotDrive != null && robot.robotDrive instanceof SwerveDrive)
                    {
                        if (pressed)
                        {
                            swerveCalibrating = false;
                            endSwerveCalibration();
                        }
                        processed = true;
                    }
                    break;

                case FrcXboxController.BUTTON_X:
                    if (testChoices.getTest() == Test.DEBUG_SWERVE_STEERING &&
                        robot.robotDrive != null && robot.robotDrive instanceof SwerveDrive)
                    {
                        if (pressed)
                        {
                            robot.robotDrive.syncSteerEncoders(true);
                        }
                        processed = true;
                    }
                    break;

                case FrcXboxController.BUTTON_Y:
                    if (testChoices.getTest() == Test.DEBUG_SWERVE_STEERING &&
                        robot.robotDrive != null && robot.robotDrive instanceof SwerveDrive)
                    {
                        if (pressed)
                        {
                            testSwerveSteering = !testSwerveSteering;
                            if (testSwerveSteering)
                            {
                                swerveSteerAngle = 0.0;
                                nextSteerAngleTime = TrcTimer.getCurrentTime();
                            }
                        }
                        processed = true;
                    }
                    break;

                case FrcXboxController.BACK:
                    // Test auto balance from inside the community.
                    if (testChoices.getTest() == Test.SUBSYSTEMS_TEST && robot.robotDrive != null)
                    {
                        if (pressed)
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
                        processed = true;
                    }
                    break;
    
                case FrcXboxController.START:
                    // Test auto balance from outside the community.
                    if (testChoices.getTest() == Test.SUBSYSTEMS_TEST && robot.robotDrive != null)
                    {
                        if (pressed)
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
                        processed = true;
                    }
                    break;
            }
            //
            // If the control was not processed by this method, pass it back to TeleOp.
            //
            if (!processed)
            {
                super.driverControllerButtonEvent(button, pressed);
            }
        }
    }   //driverControllerButtonEvent

    /**
     * This method sets the PID coefficients of the given PID Actuator to values entered from ShuffleBoard for PID
     * tuning. If there was already a previous tuning PID Actuator, its PID coefficients are restored before enabling
     * the tuning of the new PID Actuator.
     */
    private void tunePidActuator(TrcPidActuator pidActuator)
    {
        if (tunePidCtrl != null && savedPidCoeffs != null)
        {
            tunePidCtrl.setPidCoefficients(savedPidCoeffs);
        }

        tunePidCtrl = pidActuator.getPidController();
        savedPidCoeffs = tunePidCtrl.getPidCoefficients();
        tunePidCtrl.setPidCoefficients(testChoices.getTunePidCoefficients());
    }   //tunePidActuator

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
        int lineNum = 1;
        //
        // Display drivebase info.
        //
        if (robot.robotDrive != null)
        {
            // line 1
            robot.dashboard.displayPrintf(
                lineNum, "DriveBase: Pose=%s,Vel=%s",
                robot.robotDrive.driveBase.getFieldPosition(), robot.robotDrive.driveBase.getFieldVelocity());
            lineNum++;
            // line 2
            robot.dashboard.displayPrintf(
                lineNum, "Drive(Pwr/Enc): lf=%.2f/%.0f,rf=%.2f/%.0f,lb=%.2f/%.0f,rb=%.2f/%.0f",
                robot.robotDrive.lfDriveMotor.getMotorPower(), robot.robotDrive.lfDriveMotor.getPosition(),
                robot.robotDrive.rfDriveMotor.getMotorPower(), robot.robotDrive.rfDriveMotor.getPosition(),
                robot.robotDrive.lbDriveMotor.getMotorPower(), robot.robotDrive.lbDriveMotor.getPosition(),
                robot.robotDrive.rbDriveMotor.getMotorPower(), robot.robotDrive.rbDriveMotor.getPosition());
            lineNum++;
            if (robot.robotDrive instanceof SwerveDrive)
            {
                SwerveDrive swerveDrive = (SwerveDrive) robot.robotDrive;
                // line 3
                robot.dashboard.displayPrintf(
                    lineNum, "Steer(Enc/Angle): lf=%.3f/%.3f,rf=%.3f/%.3f,lb=%.3f/%.3f,rb=%.3f/%.3f",
                    swerveDrive.lfSteerEncoder.getPosition(), swerveDrive.lfWheel.getSteerAngle(),
                    swerveDrive.rfSteerEncoder.getPosition(), swerveDrive.rfWheel.getSteerAngle(),
                    swerveDrive.lbSteerEncoder.getPosition(), swerveDrive.lbWheel.getSteerAngle(),
                    swerveDrive.rbSteerEncoder.getPosition(), swerveDrive.rbWheel.getSteerAngle());
                lineNum++;
            }
        }
        //
        // Display other subsystems and sensor info.
        //
        if (robot.elevator != null)
        {
            // line 4
            robot.dashboard.displayPrintf(lineNum, robot.elevator.toString());
            lineNum++;
        }

        if (robot.arm != null)
        {
            // line 5
            robot.dashboard.displayPrintf(lineNum, robot.arm.toString());
            lineNum++;
        }

        if (robot.wrist != null)
        {
            // line 6
            robot.dashboard.displayPrintf(lineNum, robot.wrist.toString());
            lineNum++;
        }

        if (robot.intake != null)
        {
            // line 7
            robot.dashboard.displayPrintf(lineNum, robot.intake.toString());
            lineNum++;
        }

        if (robot.battery != null)
        {
            // line 8
            robot.dashboard.displayPrintf(lineNum, robot.battery.toString());
            lineNum++;
        }
    }   //displaySensorStates

    /**
     * This method is called periodically to use vision to perform objects detection. It updates the dashboard with
     * the information of the detected objects.
     */
    private void doVisionTest()
    {
        int lineNum = 1;

        if (robot.photonVision != null)
        {
            PipelineType pipelineType = robot.photonVision.getPipeline();
            FrcPhotonVision.DetectedObject targetInfo = robot.photonVision.getBestDetectedObject();

            if (targetInfo != null)
            {
                // line 1
                robot.dashboard.displayPrintf(
                    lineNum, "PhotonVision[%s]: timestamp=%.3f, targetInfo=%s",
                    pipelineType, targetInfo.timestamp, targetInfo);
                lineNum++;

                // line 2
                robot.dashboard.displayPrintf(lineNum, "TargetPoseFrom2D: %s", targetInfo.targetPoseFrom2D);
                lineNum++;

                // line 3
                robot.dashboard.displayPrintf(
                    lineNum, "TargetPoseFrom3D: %s", targetInfo.targetPoseFrom3D);
                lineNum++;

                TrcPose2D robotPose =
                    robot.photonVision.getEstimatedFieldPosition(robot.robotDrive.driveBase.getFieldPosition());
                if (robotPose != null)
                {
                    // line 4
                    robot.dashboard.displayPrintf(lineNum, "RobotEstimatedPose: %s", robotPose);
                    lineNum++;
                }

                robotPose = robot.photonVision.getRobotFieldPosition(targetInfo);
                if (robotPose != null)
                {
                    // line 5
                    robot.dashboard.displayPrintf(lineNum, "RobotPose: %s", robotPose);
                    lineNum++;
                }
            }
        }
        else if (robot.openCvVision != null)
        {
            TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> targetInfo =
                robot.openCvVision.getTargetInfo(null, null);

            if (targetInfo != null)
            {
                // line 1
                robot.dashboard.displayPrintf(
                    lineNum, "OpenCvVision[%s]: targetInfo=%s", robot.openCvVision.getPipeline(), targetInfo);
                lineNum++;
            }
        }
    }   //doVisionTest

    /**
     * This methods starts Swerve steering alignment calibration.
     */
    private void startSwerveCalibration()
    {
        for (int i = 0; i < steerZeros.length; i++)
        {
            steerZeros[i] = 0.0;
        }
        steerZeroSumCount = 0;
    }   //startSwerveCalibration

    /**
     * This methods ends Swerve steering alignment calibration and save the results to the steerzero file.
     */
    private void endSwerveCalibration()
    {
        for (int i = 0; i < steerZeros.length; i++)
        {
            steerZeros[i] /= steerZeroSumCount;
        }
        SwerveDrive.saveSteerZeroPositions(steerZeros);
    }   //endSwerveCalibration

}   //class FrcTest
