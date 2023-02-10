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

import java.util.Locale;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcOpenCvDetector;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobotBattery;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcVisionTargetInfo;
import TrcCommonLib.trclib.TrcRobot.RunMode;
import TrcFrcLib.frclib.FrcAHRSGyro;
import TrcFrcLib.frclib.FrcDashboard;
import TrcFrcLib.frclib.FrcJoystick;
import TrcFrcLib.frclib.FrcMatchInfo;
import TrcFrcLib.frclib.FrcPdp;
import TrcFrcLib.frclib.FrcRemoteVisionProcessor;
import TrcFrcLib.frclib.FrcRobotBase;
import TrcFrcLib.frclib.FrcRobotBattery;
import TrcFrcLib.frclib.FrcXboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

/**
 * The Main class is configured to instantiate and automatically run this class,
 * and to call the functions corresponding to each mode, as described in the TrcRobot
 * documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main class to reflect the name change.
 */
public class Robot extends FrcRobotBase
{
    //
    // Global objects.
    //
    public final FrcDashboard dashboard = FrcDashboard.getInstance();
    public final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
    private double nextDashboardUpdateTime = TrcTimer.getModeElapsedTime();
    private boolean traceLogOpened = false;

    //
    // Inputs.
    //
    public FrcJoystick leftDriveStick, rightDriveStick;
    public FrcJoystick operatorStick;
    public FrcJoystick buttonPanel;
    public FrcJoystick switchPanel;
    public FrcXboxController driverController;

    //
    // Sensors.
    //
    public FrcPdp pdp;
    public TrcRobotBattery battery;
    public AnalogInput pressureSensor;

    //
    // Miscellaneous hardware.
    //
    public LEDIndicator ledIndicator;

    //
    // Vision subsystem.
    //
    public PhotonVision photonVision;
    public LimeLightVision limeLightVision;
    public OpenCvVision openCvVision;

    //
    // DriveBase subsystem.
    //
    public RobotDrive robotDrive;

    //
    // Other subsystems.
    //
    public Grabber grabber;
    public Lift lift;
    public Arm arm;
    public Intake intake; 

    /**
     * Constructor: Create an instance of the object.
     */
    public Robot()
    {
        super(RobotParams.ROBOT_NAME);
    }   //Robot

    /**
     * This method is called when the robot is first started up and should be used for any initialization code
     * including creation and initialization of all robot hardware and subsystems.
     *
     * To create new hardware or subsystem, follow the steps below:
     * 1. Create a public class variable for the new hardware/subsystem.
     * 2. Instantiate and initialize the new hardware/subsystem object in this method.
     * 3. Put code in updateDashboard to display status of the new hardware/subsystem if necessary.
     * 4. Put code in robotStartMode or robotStopMode to configure/reset hardware/subsystem if necessary.
     * 5. Put code in FrcTeleOp to operate the subsystem if necessary (i.e. slowPeriodic/xxxButtonEvent).
     * 6. Create a getter method for the new sensor only if necessary (e.g. sensor value needs translation).
     */
    @Override
    public void robotInit()
    {
        //
        // Create and initialize global objects.
        //

        //
        // Create and initialize inputs.
        //
        if (RobotParams.Preferences.useXboxController)
        {
            driverController = new FrcXboxController("DriverController", RobotParams.XBOX_DRIVERCONTROLLER);
            driverController.setLeftYInverted(true);
        }
        else
        {
            leftDriveStick = new FrcJoystick("DriverLeftStick", RobotParams.JSPORT_DRIVER_LEFTSTICK);
            leftDriveStick.setYInverted(true);
            rightDriveStick = new FrcJoystick("DriverRightStick", RobotParams.JSPORT_DRIVER_RIGHTSTICK);
            rightDriveStick.setYInverted(true);
        }
        operatorStick = new FrcJoystick("operatorStick", RobotParams.JSPORT_OPERATORSTICK);
        operatorStick.setYInverted(false);
        if (RobotParams.Preferences.useButtonPanels)
        {
            buttonPanel = new FrcJoystick("buttonPanel", RobotParams.JSPORT_BUTTON_PANEL);
            switchPanel = new FrcJoystick("switchPanel", RobotParams.JSPORT_SWITCH_PANEL);
        }

        //
        // Create and initialize sensors.
        //
        if (RobotParams.Preferences.useStreamCamera)
        {
            UsbCamera camera = CameraServer.startAutomaticCapture("UsbWebcam", 0);
            camera.setResolution(160, 120);
            camera.setFPS(10);
        }

        if (RobotParams.Preferences.usePdp)
        {
            pdp = new FrcPdp(RobotParams.CANID_PDP, ModuleType.kRev);
            pdp.setSwitchableChannel(false);
            battery = new FrcRobotBattery(pdp);
        }

        if (RobotParams.Preferences.usePressureSensor)
        {
            pressureSensor = new AnalogInput(RobotParams.AIN_PRESSURE_SENSOR);
        }

        //
        // Create and initialize miscellaneous hardware.
        //
        ledIndicator = new LEDIndicator();

        //
        // Create and initialize Vision subsystem.
        //
        if (RobotParams.Preferences.usePhotonVision)
        {
            photonVision = new PhotonVision("trc-limelight", globalTracer);
        }

        if (RobotParams.Preferences.useLimeLightVision)
        {
            limeLightVision = new LimeLightVision("limelight", globalTracer);
        }

        if (RobotParams.Preferences.useOpenCvVision)
        {
            UsbCamera camera = CameraServer.startAutomaticCapture();
            camera.setResolution(RobotParams.CAMERA_IMAGE_WIDTH, RobotParams.CAMERA_IMAGE_HEIGHT);
            camera.setFPS(10);
            openCvVision = new OpenCvVision(
                "OpenCvVision", 1, RobotParams.CAMERA_IMAGE_WIDTH, RobotParams.CAMERA_IMAGE_HEIGHT, null, null,
                CameraServer.getVideo(),
                CameraServer.putVideo("UsbWebcam", RobotParams.CAMERA_IMAGE_WIDTH, RobotParams.CAMERA_IMAGE_HEIGHT),
                null);
        }

        if (!RobotParams.Preferences.noRobot)
        {
            //
            // Create and initialize RobotDrive subsystem.
            //
            robotDrive = RobotParams.Preferences.swerveRobot? new SwerveDrive(this): new WestCoastDrive(this);

            //
            // Create and initialize other subsystems.
            //
            if (RobotParams.Preferences.useSubsystems)
            {
                if (RobotParams.Preferences.useGrabber)
                {
                    grabber = new Grabber();
                    grabber.release();
                }
            }
        }

        if (RobotParams.Preferences.useLift) {
            lift = new Lift(this);
        }    

        if (RobotParams.Preferences.useArm) {
            arm = new Arm(this);
        }  


        //
        // Miscellaneous.
        //
        if (pdp != null)
        {
            pdp.registerEnergyUsedForAllUnregisteredChannels();
        }

        //
        // Create Robot Modes.
        //
        setupRobotModes(new FrcTeleOp(this), new FrcAuto(this), new FrcTest(this), new FrcDisabled(this));
    }   //robotInit

    /**
     * This method is called to prepare the robot before a robot mode is about to start.
     *
     * @param runMode specifies the current run mode.
     * @param prevMode specifies the previous run mode.
     */
    @Override
    public void robotStartMode(RunMode runMode, RunMode prevMode)
    {
        final String funcName = "robotStartMode";

        //
        // Read FMS Match info.
        //
        FrcMatchInfo matchInfo = FrcMatchInfo.getMatchInfo();

        //
        // Start trace logging.
        //
        if (runMode != RunMode.DISABLED_MODE && RobotParams.Preferences.useTraceLog)
        {
            openTraceLog(matchInfo);
            setTraceLogEnabled(true);
        }
        globalTracer.traceInfo(
            funcName, "[%.3f] %s: ***** %s *****", TrcTimer.getModeElapsedTime(),
            matchInfo.eventDate, runMode);

        //
        // Start subsystems.
        //
        if (robotDrive != null)
        {
            robotDrive.startMode(runMode, prevMode);
        }
        ledIndicator.reset();
    }   //robotStartMode

    /**
     * This method is called to prepare the robot right after a robot mode has been stopped.
     *
     * @param runMode specifies the current run mode.
     * @param nextMode specifies the next run mode.
     */
    @Override
    public void robotStopMode(RunMode runMode, RunMode nextMode)
    {
        final String funcName = "robotStopMode";

        //
        // Stop subsystems.
        //
        if (robotDrive != null)
        {
            robotDrive.stopMode(runMode, nextMode);
        }

        if (RobotParams.Preferences.useSubsystems)
        {
        }
        ledIndicator.reset();

        //
        // Performance status report.
        //
        if (battery != null)
        {
            double totalEnergy = battery.getTotalEnergy();
            globalTracer.traceInfo(
                funcName, "TotalEnergy=%.3fWh (%.2f%%)",
                totalEnergy, totalEnergy * 100.0 / RobotParams.BATTERY_CAPACITY_WATT_HOUR);
        }

        if (runMode != RunMode.DISABLED_MODE)
        {
            printPerformanceMetrics(globalTracer);
        }

        //
        // Stop trace logging.
        //
        setTraceLogEnabled(false);
        closeTraceLog();
    }   //robotStopMode

    /**
     * This method is called periodically to update various hardware/subsystem status of the robot to the dashboard
     * and trace log. In order to lower the potential impact these updates, this method will only update the dashboard
     * at DASHBOARD_UPDATE_INTERVAL.
     */
    public void updateStatus()
    {
        final String funcName = "updateStatus";
        double currTime = TrcTimer.getModeElapsedTime();
        RunMode runMode = getCurrentRunMode();

        if (currTime >= nextDashboardUpdateTime)
        {
            nextDashboardUpdateTime = currTime + RobotParams.DASHBOARD_UPDATE_INTERVAL;

            if (RobotParams.Preferences.debugPowerConsumption)
            {
                if (pdp != null)
                {
                    dashboard.putNumber("Power/pdpTotalCurrent", pdp.getTotalCurrent());
                    dashboard.putNumber("Power/totalEnergy", battery.getTotalEnergy());
                    dashboard.putData("Power/pdpInfo", pdp.getPdpSendable());
                    if (runMode == RunMode.TELEOP_MODE)
                    {
                        globalTracer.traceInfo(
                            funcName, "[%.3f] Battery: currVoltage=%.2f, lowestVoltage=%.2f",
                            currTime, battery.getVoltage(), battery.getLowestVoltage());
                        globalTracer.traceInfo(funcName, "[%.3f] Total=%.2fA", currTime, pdp.getTotalCurrent());
                    }
                }
            }

            if (RobotParams.Preferences.debugDriveBase)
            {
                if (robotDrive != null)
                {
                    TrcPose2D robotPose = robotDrive.driveBase.getFieldPosition();

                    dashboard.putNumber("DriveBase/xPos", robotPose.x);
                    dashboard.putNumber("DriveBase/yPos", robotPose.y);
                    dashboard.putData("DriveBase/heading", ((FrcAHRSGyro) robotDrive.gyro).getGyroSendable());
                    dashboard.putNumber("DriveBase/Yaw", ((FrcAHRSGyro) robotDrive.gyro).ahrs.getYaw());
                    dashboard.putNumber("DriveBase/Pitch", ((FrcAHRSGyro) robotDrive.gyro).ahrs.getPitch());
                    dashboard.putNumber("DriveBase/Roll", ((FrcAHRSGyro) robotDrive.gyro).ahrs.getRoll());
                    dashboard.putNumber("DriveBase/AccelX", ((FrcAHRSGyro) robotDrive.gyro).ahrs.getWorldLinearAccelX());
                    dashboard.putNumber("DriveBase/AccelY", ((FrcAHRSGyro) robotDrive.gyro).ahrs.getWorldLinearAccelY());
                    dashboard.putNumber("DriveBase/AccelZ", ((FrcAHRSGyro) robotDrive.gyro).ahrs.getWorldLinearAccelZ());
                    dashboard.putNumber("DriverBase/Compass", ((FrcAHRSGyro) robotDrive.gyro).ahrs.getCompassHeading());

                    //
                    // DriveBase debug info.
                    //
                    double lfDriveEnc = robotDrive.lfDriveMotor.getPosition();
                    double rfDriveEnc = robotDrive.rfDriveMotor.getPosition();
                    double lbDriveEnc = robotDrive.lbDriveMotor.getPosition();
                    double rbDriveEnc = robotDrive.rbDriveMotor.getPosition();

                    dashboard.displayPrintf(
                        8, "DriveBase: lf=%.0f, rf=%.0f, lb=%.0f, rb=%.0f, avg=%.0f",
                        lfDriveEnc, rfDriveEnc, lbDriveEnc, rbDriveEnc,
                        (lfDriveEnc + rfDriveEnc + lbDriveEnc + rbDriveEnc) / 4.0);
                    dashboard.displayPrintf(9, "DriveBase: pose=%s", robotPose);

                    if (RobotParams.Preferences.debugPurePursuitDrive)
                    {
                        int lineNum = 10;
                        robotDrive.purePursuitDrive.getXPosPidCtrl().displayPidInfo(lineNum);
                        lineNum += 2;
                        robotDrive.purePursuitDrive.getYPosPidCtrl().displayPidInfo(lineNum);
                        lineNum += 2;
                        robotDrive.purePursuitDrive.getTurnPidCtrl().displayPidInfo(lineNum);
                    }
                    else if (RobotParams.Preferences.debugPidDrive)
                    {
                        int lineNum = 10;
                        if (robotDrive.pidDrive.getXPidCtrl() != null)
                        {
                            robotDrive.pidDrive.getXPidCtrl().displayPidInfo(lineNum);
                            lineNum += 2;
                        }
                        robotDrive.pidDrive.getYPidCtrl().displayPidInfo(lineNum);
                        lineNum += 2;
                        robotDrive.pidDrive.getTurnPidCtrl().displayPidInfo(lineNum);
                    }
                }
            }

            if (RobotParams.Preferences.showVisionStatus)
            {
                if (limeLightVision != null)
                {
                    FrcRemoteVisionProcessor.RelativePose pose = limeLightVision.getLastPose();

                    if (pose != null)
                    {
                        double horiAngle = limeLightVision.getTargetHorizontalAngle();
                        double vertAngle = limeLightVision.getTargetVerticalAngle();
                        double distanceToTarget = limeLightVision.getTargetDistance();
                        dashboard.putNumber("Camera/distance", distanceToTarget);
                        dashboard.putNumber("Camera/horiAngle", horiAngle);
                        dashboard.putNumber("Camera/vertAngle", vertAngle);
                        if (RobotParams.Preferences.debugVision)
                        {
                            dashboard.displayPrintf(
                                15, "VisionTarget: x=%.1f,y=%.1f,depth=%.1f/%.1f,horiAngle=%.1f,vertAngle=%.1f",
                                pose.x, pose.y, pose.r, distanceToTarget, horiAngle, vertAngle);
                        }
                    }
                    else if (RobotParams.Preferences.debugVision)
                    {
                        dashboard.displayPrintf(15, "VisionTarget: No target found!");
                    }
                }

                if (openCvVision != null)
                {
                    TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> targetInfo =
                        openCvVision.getTargetInfo(null, null);

                    if (targetInfo != null)
                    {
                        if (RobotParams.Preferences.debugVision)
                        {
                            dashboard.displayPrintf(
                                14, "%s_Info: %s", openCvVision.getDetectObjectType(), targetInfo);
                        }
                    }
                }
            }

            if (RobotParams.Preferences.showSubsystemStatus)
            {
            }
        }
    }   //updateStatus

    /**
     * This method creates and opens the trace log with the file name derived from the given match info.
     * Note that the trace log is disabled after it is opened. The caller must explicitly call setTraceLogEnabled
     * to enable/disable it.
     *
     * @param matchInfo specifies the match info from which the trace log file name is derived.
     */
    public void openTraceLog(FrcMatchInfo matchInfo)
    {
        if (RobotParams.Preferences.useTraceLog && !traceLogOpened)
        {
            String fileName = matchInfo.eventName != null?
                String.format(Locale.US, "%s_%s%03d", matchInfo.eventName, matchInfo.matchType, matchInfo.matchNumber):
                getCurrentRunMode().name();

            traceLogOpened = globalTracer.openTraceLog(RobotParams.TEAM_FOLDER + "/tracelogs", fileName);
        }
    }   //openTraceLog

    /**
     * This method closes the trace log if it was opened.
     */
    public void closeTraceLog()
    {
        if (traceLogOpened)
        {
            globalTracer.closeTraceLog();
            traceLogOpened = false;
        }
    }   //closeTraceLog

    /**
     * This method enables/disables the trace log.
     *
     * @param enabled specifies true to enable trace log, false to disable.
     */
    public void setTraceLogEnabled(boolean enabled)
    {
        if (traceLogOpened)
        {
            globalTracer.setTraceLogEnabled(enabled);
        }
    }   //setTraceLogEnabled

    //
    // Getters for sensor data.
    //

    /**
     * This method returns the pressure value from the pressure sensor.
     *
     * @return pressure value.
     */
    public double getPressure()
    {
        return (pressureSensor.getVoltage() - 0.5) * 50.0;
    }   //getPressure

}   //class Robot
