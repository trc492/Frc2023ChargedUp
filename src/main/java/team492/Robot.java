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

import java.util.Locale;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SensorCollection;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcIntake;
import TrcCommonLib.trclib.TrcOpenCvDetector;
import TrcCommonLib.trclib.TrcPidActuator;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobotBattery;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcVisionTargetInfo;
import TrcCommonLib.trclib.TrcRobot.RunMode;
import TrcFrcLib.frclib.FrcAnalogEncoder;
import TrcFrcLib.frclib.FrcCANFalcon;
import TrcFrcLib.frclib.FrcCANTalon;
import TrcFrcLib.frclib.FrcDashboard;
import TrcFrcLib.frclib.FrcJoystick;
import TrcFrcLib.frclib.FrcMatchInfo;
import TrcFrcLib.frclib.FrcPdp;
import TrcFrcLib.frclib.FrcPhotonVision;
import TrcFrcLib.frclib.FrcRemoteVisionProcessor;
import TrcFrcLib.frclib.FrcRobotBase;
import TrcFrcLib.frclib.FrcRobotBattery;
import TrcFrcLib.frclib.FrcXboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import team492.FrcAuto.ObjectType;
import team492.autotasks.TaskAutoBalance;
import team492.autotasks.TaskAutoPickup;
import team492.autotasks.TaskAutoScore;
import team492.drivebases.SwerveDrive;
import team492.subsystems.Arm;
import team492.subsystems.Elevator;
import team492.subsystems.Intake;
import team492.subsystems.LEDIndicator;
import team492.subsystems.WeedWhacker;
import team492.subsystems.Wrist;
import team492.vision.LimeLightVision;
import team492.vision.OpenCvVision;
import team492.vision.PhotonVision;

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
    private double nextDashboardUpdateTime = TrcTimer.getCurrentTime();
    private boolean traceLogOpened = false;
    //
    // Inputs.
    //
    public FrcJoystick leftDriveStick, rightDriveStick;
    public FrcJoystick operatorStick;
    public FrcJoystick buttonPanel;
    public FrcJoystick switchPanel;
    public FrcXboxController driverController;
    public FrcXboxController operatorController;
    //
    // Sensors.
    //
    public FrcPdp pdp;
    public TrcRobotBattery battery;
    public AnalogInput pressureSensor;
    // For debugging Swerve steering.
    private FrcAnalogEncoder lfSteerEnc, rfSteerEnc, lbSteerEnc, rbSteerEnc;
    private double lfSteerEncSum, rfSteerEncSum, lbSteerEncSum, rbSteerEncSum;
    private long encSumCount;
    private FrcCANFalcon lfDriveMotor, rfDriveMotor, lbDriveMotor, rbDriveMotor;
    // For debugging arm encoder.
    private FrcCANTalon armMotor;
    private SensorCollection sensorCollection;
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
    public SwerveDrive robotDrive;
    //
    // Other subsystems.
    //
    public Elevator elevator;
    public TrcPidActuator elevatorPidActuator;
    public Arm arm;
    public TrcPidActuator armPidActuator;
    public Wrist wrist;
    public TrcPidActuator wristPidActuator;
    public TrcIntake intake;
    public WeedWhacker weedWhacker;
    
    public TaskAutoScore autoScoreTask;
    public TaskAutoPickup autoPickupTask;
    public TaskAutoBalance autoBalanceTask;

    public int scoreLevel = 2;
    public ObjectType objType = ObjectType.CONE;

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
        if (RobotParams.Preferences.useDriverXboxController)
        {
            driverController = new FrcXboxController("DriverController", RobotParams.XBOX_DRIVER_CONTROLLER);
            driverController.setLeftYInverted(true);
            driverController.setRightYInverted(true);
        }
        else
        {
            leftDriveStick = new FrcJoystick("DriverLeftStick", RobotParams.JSPORT_DRIVER_LEFTSTICK);
            leftDriveStick.setYInverted(true);
            rightDriveStick = new FrcJoystick("DriverRightStick", RobotParams.JSPORT_DRIVER_RIGHTSTICK);
            rightDriveStick.setYInverted(true);
        }

        operatorStick = new FrcJoystick("operatorStick", RobotParams.JSPORT_OPERATORSTICK);
        operatorStick.setYInverted(true);

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

        // if (RobotParams.Preferences.usePressureSensor)
        // {
        //     pressureSensor = new AnalogInput(RobotParams.AIN_PRESSURE_SENSOR);
        // }

        //
        // Create and initialize miscellaneous hardware.
        //
        ledIndicator = new LEDIndicator();
        //
        // Create and initialize Vision subsystem.
        //
        if (RobotParams.Preferences.usePhotonVision)
        {
            photonVision = new PhotonVision("OV5647", ledIndicator, null);
        }

        if (RobotParams.Preferences.useLimeLightVision)
        {
            limeLightVision = new LimeLightVision("limelight", null);
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
            robotDrive = new SwerveDrive(this);
            //
            // Create and initialize other subsystems.
            //
            if (RobotParams.Preferences.useSubsystems)
            {
                if (RobotParams.Preferences.useElevator)
                {
                    elevator = new Elevator(globalTracer);
                    elevatorPidActuator = elevator.getPidActuator();
                }

                if (RobotParams.Preferences.useArm)
                {
                    arm = new Arm(globalTracer);
                    armPidActuator = arm.getPidActuator();
                }

                if (RobotParams.Preferences.useWrist)
                {
                    wrist = new Wrist(this, globalTracer);
                    wristPidActuator = wrist.getPidActuator();
                }

                if (RobotParams.Preferences.useIntake)
                {
                    TrcIntake.Parameters intakeParams = new TrcIntake.Parameters()
                        .setTriggerInverted(RobotParams.INTAKE_TRIGGER_INVERTED)
                        .setMsgTracer(globalTracer);
                    intake = new Intake(this, globalTracer, intakeParams).getTrcIntake();
                }

                if (RobotParams.Preferences.useWeedWhacker)
                {
                    weedWhacker = new WeedWhacker(this, globalTracer);
                }

                autoScoreTask = new TaskAutoScore("TaskAutoScore", this, globalTracer);
                autoPickupTask = new TaskAutoPickup("TaskAutoPickup", this, globalTracer);
                autoBalanceTask = new TaskAutoBalance("TaskAutoBalance", this, globalTracer);
            }
        }
        //
        // Miscellaneous.
        //
        if (pdp != null)
        {
            pdp.registerEnergyUsedForAllUnregisteredChannels();
        }

        if (RobotParams.Preferences.debugSwerveSteering)
        {
            lfDriveMotor = new FrcCANFalcon("lfDriveMotor", RobotParams.CANID_LEFTFRONT_DRIVE);
            rfDriveMotor = new FrcCANFalcon("rfDriveMotor", RobotParams.CANID_RIGHTFRONT_DRIVE);
            lbDriveMotor = new FrcCANFalcon("lbDriveMotor", RobotParams.CANID_LEFTBACK_DRIVE);
            rbDriveMotor = new FrcCANFalcon("rbDriveMotor", RobotParams.CANID_RIGHTBACK_DRIVE);
            lfDriveMotor.setMotorInverted(false);
            rfDriveMotor.setMotorInverted(true);
            lbDriveMotor.setMotorInverted(false);
            rbDriveMotor.setMotorInverted(true);
            lfSteerEnc = new FrcAnalogEncoder("lfSteerEnc", RobotParams.AIN_LEFTFRONT_STEER_ENCODER);
            rfSteerEnc = new FrcAnalogEncoder("rfSteerEnc", RobotParams.AIN_RIGHTFRONT_STEER_ENCODER);
            lbSteerEnc = new FrcAnalogEncoder("lbSteerEnc", RobotParams.AIN_LEFTBACK_STEER_ENCODER);
            rbSteerEnc = new FrcAnalogEncoder("rbSteerEnc", RobotParams.AIN_RIGHTBACK_STEER_ENCODER);
            lfSteerEnc.setInverted(true);
            rfSteerEnc.setInverted(true);
            lbSteerEnc.setInverted(true);
            rbSteerEnc.setInverted(true);
            double[] zeros = SwerveDrive.getSteerZeroPositions();
            dashboard.displayPrintf(
                8, "SteerZeros: lf:%.3f, rf:%.3f, lb:%.3f, rb:%.3f", zeros[0], zeros[1], zeros[2], zeros[3]);
            lfSteerEnc.setScaleAndOffset(360.0, zeros[0]);
            rfSteerEnc.setScaleAndOffset(360.0, zeros[1]);
            lbSteerEnc.setScaleAndOffset(360.0, zeros[2]);
            rbSteerEnc.setScaleAndOffset(360.0, zeros[3]);
            lfSteerEnc.setEnabled(true);
            rfSteerEnc.setEnabled(true);
            lbSteerEnc.setEnabled(true);
            rbSteerEnc.setEnabled(true);
            lfSteerEncSum = rfSteerEncSum = lbSteerEncSum = rbSteerEncSum = 0.0;
            encSumCount = 0;
        }

        if (RobotParams.Preferences.debugArmEncoder)
        {
            armMotor = new FrcCANTalon("armMotor", RobotParams.CANID_ARM);
            sensorCollection = armMotor.motor.getSensorCollection();
            sensorCollection.syncQuadratureWithPulseWidth(0, 4095, false, -1650, 10);
            armMotor.motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
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
            openTraceLog(matchInfo, runMode);
            setTraceLogEnabled(true);
        }
        globalTracer.traceInfo(
            funcName, "%s: [%.3f] ***** Start %s *****", matchInfo.eventDate, TrcTimer.getModeElapsedTime(), runMode);
        //
        // Start subsystems.
        //
        if (robotDrive != null)
        {
            robotDrive.startMode(runMode, prevMode);
        }

        if (elevator != null)
        {
            elevatorPidActuator.getMotor().setBrakeModeEnabled(runMode != RunMode.DISABLED_MODE);
        }

        if (arm != null)
        {
            armPidActuator.getMotor().setBrakeModeEnabled(runMode != RunMode.DISABLED_MODE);
        }

        if (wrist != null)
        {
            wristPidActuator.getMotor().setBrakeModeEnabled(runMode != RunMode.DISABLED_MODE);
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

        globalTracer.traceInfo(funcName, "[%.3f] ***** Stop %s *****", TrcTimer.getModeElapsedTime(), runMode);
        //
        // Stop subsystems.
        //
        if (robotDrive != null)
        {
            robotDrive.stopMode(runMode, nextMode);
        }

        // if (RobotParams.Preferences.useSubsystems)
        // {
        // }
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
        double currTime = TrcTimer.getCurrentTime();
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
            else if (RobotParams.Preferences.debugVision)
            {
                if (RobotParams.Preferences.debugPhoton && photonVision != null)
                {
                    FrcPhotonVision.DetectedObject targetInfo = photonVision.getBestDetectedObject();

                    if (targetInfo != null)
                    {
                        // robot.globalTracer.traceInfo("doVisionTest", "Photon: %s", targetInfo);
                        dashboard.displayPrintf(9, "Photon: %s", targetInfo);

                        TrcPose2D robotPose = photonVision.getRobotFieldPosition(targetInfo);
                        if (robotPose != null)
                        {
                            dashboard.displayPrintf(9, "RobotPose: %s", robotPose);
                        }
                    }
                }
                else if (RobotParams.Preferences.debugLimeLight && limeLightVision != null)
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
                                9, "VisionTarget: x=%.1f,y=%.1f,depth=%.1f/%.1f,horiAngle=%.1f,vertAngle=%.1f",
                                pose.x, pose.y, pose.r, distanceToTarget, horiAngle, vertAngle);
                        }
                    }
                    else if (RobotParams.Preferences.debugVision)
                    {
                        dashboard.displayPrintf(9, "VisionTarget: No target found!");
                    }
                }
                else if (RobotParams.Preferences.debugOpenCv && openCvVision != null)
                {
                    TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> targetInfo =
                        openCvVision.getTargetInfo(null, null);

                    if (targetInfo != null)
                    {
                        if (RobotParams.Preferences.debugVision)
                        {
                            dashboard.displayPrintf(
                                9, "%s_Info: %s", openCvVision.getDetectObjectType(), targetInfo);
                        }
                    }
                }
            }
            else if (RobotParams.Preferences.debugDriveBase)
            {
                if (robotDrive != null)
                {
                    int lineNum = 8;
                    //
                    // DriveBase debug info.
                    //
                    double lfDriveEnc = robotDrive.lfDriveMotor.getPosition();
                    double rfDriveEnc = robotDrive.rfDriveMotor.getPosition();
                    double lbDriveEnc = robotDrive.lbDriveMotor.getPosition();
                    double rbDriveEnc = robotDrive.rbDriveMotor.getPosition();

                    dashboard.displayPrintf(
                        lineNum, "DriveEncPos: lf=%.0f, rf=%.0f, lb=%.0f, rb=%.0f, avg=%.0f",
                        lfDriveEnc, rfDriveEnc, lbDriveEnc, rbDriveEnc,
                        (lfDriveEnc + rfDriveEnc + lbDriveEnc + rbDriveEnc) / 4.0);
                    lineNum++;

                    if (robotDrive instanceof SwerveDrive)
                    {
                        SwerveDrive swerveDrive = (SwerveDrive) robotDrive;
                        dashboard.displayPrintf(
                            lineNum, "SteerPos(Deg/Enc/Abs): lf=%.1f/%.0f/%.3f,rf=%.1f/%.0f/%.3f,lb=%.1f/%.0f/%.3f,rb=%.1f/%.0f/%.3f",
                            swerveDrive.lfWheel.getSteerAngle(), swerveDrive.lfSteerMotor.getPosition(), swerveDrive.lfSteerEncoder.getRawPosition(),
                            swerveDrive.rfWheel.getSteerAngle(), swerveDrive.rfSteerMotor.getPosition(), swerveDrive.rfSteerEncoder.getRawPosition(),
                            swerveDrive.lbWheel.getSteerAngle(), swerveDrive.lbSteerMotor.getPosition(), swerveDrive.lbSteerEncoder.getRawPosition(),
                            swerveDrive.rbWheel.getSteerAngle(), swerveDrive.rbSteerMotor.getPosition(), swerveDrive.rbSteerEncoder.getRawPosition());
                        lineNum++;
                    }

                    if (RobotParams.Preferences.debugPurePursuitDrive)
                    {
                        robotDrive.purePursuitDrive.getXPosPidCtrl().displayPidInfo(lineNum);
                        lineNum += 2;
                        robotDrive.purePursuitDrive.getYPosPidCtrl().displayPidInfo(lineNum);
                        lineNum += 2;
                        robotDrive.purePursuitDrive.getTurnPidCtrl().displayPidInfo(lineNum);
                    }
                    else if (RobotParams.Preferences.debugPidDrive)
                    {
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
            else if (RobotParams.Preferences.debugSubsystems)
            {
                int lineNum = 9;

                if (elevator != null)
                {
                    dashboard.displayPrintf(lineNum, elevator.toString());
                    lineNum++;
                }

                if (arm != null)
                {
                    dashboard.displayPrintf(lineNum, arm.toString());
                    lineNum++;
                }

                if (wrist != null)
                {
                    dashboard.displayPrintf(lineNum, wrist.toString());
                    lineNum++;
                }

                if (intake != null)
                {
                    dashboard.displayPrintf(lineNum, intake.toString());
                    lineNum++;
                }

                // if (photonVision != null && photonVision.getPipeline() == PipelineType.APRILTAG)
                // {
                //     FrcPhotonVision.DetectedObject detectedObj = photonVision.getBestDetectedObject();
                //     TrcPose2D robotPose = detectedObj != null? photonVision.getRobotFieldPosition(detectedObj): null;
                //     dashboard.displayPrintf(lineNum, "Vision[APRILTAG]: robotPose=%s", robotPose);
                //     lineNum++;
                // }

                // if (robotDrive != null)
                // {
                //     double[] drivePwr = robotDrive.getDriveInputs();
                //     dashboard.displayPrintf(
                //         lineNum, "Drive: Pwr:%.3f,%.3f,%.3f", drivePwr[0],drivePwr[1],drivePwr[2]);
                //     lineNum++;
                //     dashboard.displayPrintf(
                //         lineNum, "Gyro: Yaw=%.3f, Pitch=%.3f, Roll=%.3f",
                //         robotDrive.driveBase.getHeading(), robotDrive.getGyroPitch(), robotDrive.getGyroRoll());
                //     lineNum++;
                //     dashboard.putNumber("Graphs/GyroRoll", robotDrive.getGyroRoll());
                // }
            }
            else if (RobotParams.Preferences.debugSwerveSteering)
            {
                double lfRawEnc = lfSteerEnc.getRawPosition();
                double rfRawEnc = rfSteerEnc.getRawPosition();
                double lbRawEnc = lbSteerEnc.getRawPosition();
                double rbRawEnc = rbSteerEnc.getRawPosition();
                lfSteerEncSum += lfRawEnc;
                rfSteerEncSum += rfRawEnc;
                lbSteerEncSum += lbRawEnc;
                rbSteerEncSum += rbRawEnc;
                encSumCount++;
                dashboard.displayPrintf(8, "SteerEncVolt(%.3f): lf:%.3f, rf:%.3f, lb:%.3f, rb:%.3f",
                    RobotController.getVoltage5V(), lfSteerEnc.getRawVoltage(), rfSteerEnc.getRawVoltage(),
                    lbSteerEnc.getRawVoltage(), rbSteerEnc.getRawVoltage());
                dashboard.displayPrintf(
                    9, "SteerEncRaw: lf=%.3f/%f, rf=%.3f/%f, lb=%.3f/%f, rb=%.3f/%f",
                    lfRawEnc, lfSteerEncSum/encSumCount, rfRawEnc, rfSteerEncSum/encSumCount,
                    lbRawEnc, lbSteerEncSum/encSumCount, rbRawEnc, rbSteerEncSum/encSumCount);
                dashboard.displayPrintf(
                    10, "SteerEncPos: lf=%.3f, rf=%.3f, lb=%.3f, rb=%.3f",
                    lfSteerEnc.getPosition(), rfSteerEnc.getPosition(),
                    lbSteerEnc.getPosition(), rbSteerEnc.getPosition());
                dashboard.displayPrintf(
                    11, "DriveMotorPos: lf=%.3f, rf=%.3f, lb=%.3f, rb=%.3f",
                    lfDriveMotor.getPosition(), rfDriveMotor.getPosition(),
                    lbDriveMotor.getPosition(), rbDriveMotor.getPosition());
            }
            else if (RobotParams.Preferences.debugArmEncoder)
            {
                dashboard.displayPrintf(
                    8, "ArmEnc: pwmPos=%d, quadPos=%d, sensorPos=%.1f",
                    sensorCollection.getQuadraturePosition(), sensorCollection.getQuadraturePosition(),
                    armMotor.motor.getSelectedSensorPosition());
                dashboard.displayPrintf(
                    9, "ArmLimitSW: Rev=%s, Fwd=%s",
                    armMotor.isRevLimitSwitchActive(), armMotor.isFwdLimitSwitchActive());
            }
        }
    }   //updateStatus

    /**
     * This method creates and opens the trace log with the file name derived from the given match info.
     * Note that the trace log is disabled after it is opened. The caller must explicitly call setTraceLogEnabled
     * to enable/disable it.
     *
     * @param matchInfo specifies the match info from which the trace log file name is derived.
     * @param runMode specifies the current run mode.
     */
    public void openTraceLog(FrcMatchInfo matchInfo, RunMode runMode)
    {
        if (RobotParams.Preferences.useTraceLog && !traceLogOpened)
        {
            String fileName = matchInfo.eventName != null?
                String.format(
                    Locale.US, "%s_%s%03d_%s",
                    matchInfo.eventName, matchInfo.matchType, matchInfo.matchNumber, runMode):
                runMode.name();

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

    private final TrcEvent elevatorEvent = new TrcEvent("robot.elevatorEvent");
    private final TrcEvent armEvent = new TrcEvent("robot.armEvent");
    private final TrcEvent wristEvent = new TrcEvent("robot.wristEvent");
    private TrcEvent completionEvent = null;

    /**
     * This method is called when one of the subsystem operations has completed. It signals the completion event only
     * if all three subsystem operations have completed.
     *
     * @param context not used.
     */
    private void prepCompletion(Object context)
    {
        if (completionEvent != null && elevatorEvent.isSignaled() && armEvent.isSignaled() && wristEvent.isSignaled())
        {
            completionEvent.signal();
            completionEvent = null;
        }
    }   //prepCompletion

    /**
     * This method prep the subsystems for a certain operation.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystems.
     * @param elevatorDelay specifies the delay in seconds before moving elevator.
     * @param elevatorPos specifies the elevator position.
     * @param armDelay specifies the delay in seconds before moving arm.
     * @param armPos specifies the arm position.
     * @param wristDelay specifies the delay in seconds before moving wrist.
     * @param wristPos specifies the wrist position.
     * @param timeout specifies the maximum time allowed for the operation.
     * @param event specifies the event to signal when the operation is completed.
     */
    public void prepSubsystems(
        String owner, double elevatorDelay, double elevatorPos, double armDelay, double armPos, double wristDelay,
        double wristPos, double timeout, TrcEvent event)
    {
        if (elevator != null && arm != null && wrist != null)
        {
            if (event != null)
            {
                elevatorEvent.clear();
                armEvent.clear();
                wristEvent.clear();
                elevatorEvent.setCallback(this::prepCompletion, null);
                armEvent.setCallback(this::prepCompletion, null);
                wristEvent.setCallback(this::prepCompletion, null);
                completionEvent = event;
            }

            elevatorPidActuator.setPosition(
                owner, elevatorDelay, elevatorPos, true, 1.0, event != null? elevatorEvent: null, timeout);
            armPidActuator.setPosition(
                owner, armDelay, armPos, true, RobotParams.ARM_MAX_POWER, event != null? armEvent: null, timeout);
            wristPidActuator.setPosition(
                owner, wristDelay, wristPos, true, RobotParams.WRIST_MAX_POWER, event != null? wristEvent: null,
                timeout);
        }
    }   //prepSubsystems

    /**
     * This method prep the subsystems for a certain operation.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystems.
     * @param elevatorPos specifies the elevator position.
     * @param armPos specifies the arm position.
     * @param wristPos specifies the wrist position.
     * @param timeout specifies the maximum time allowed for the operation.
     * @param event specifies the event to signal when the operation is completed.
     */
    public void prepSubsystems(
        String owner, double elevatorPos, double armPos, double wristPos, double timeout, TrcEvent event)
    {
        prepSubsystems(owner, 0.0, elevatorPos, 0.0, armPos, 0.0, wristPos, timeout, event);
    }   //prepSubsystems

    /**
     * This method prep the subsystems for a certain operation.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystems.
     * @param elevatorDelay specifies the delay in seconds before moving elevator.
     * @param elevatorPos specifies the elevator position.
     * @param armDelay specifies the delay in seconds before moving arm.
     * @param armPos specifies the arm position.
     * @param wristDelay specifies the delay in seconds before moving wrist.
     * @param wristPos specifies the wrist position.
     * @param timeout specifies the maximum time allowed for the operation.
     */
    public void prepSubsystems(
        String owner, double elevatorDelay, double elevatorPos, double armDelay, double armPos, double wristDelay,
        double wristPos, double timeout)
    {
        prepSubsystems(owner, elevatorDelay, elevatorPos, armDelay, armPos, wristDelay, wristPos, timeout, null);
    }   //prepSubsystems

    /**
     * This method prep the subsystems for a certain operation.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystems.
     * @param elevatorDelay specifies the delay in seconds before moving elevator.
     * @param elevatorPos specifies the elevator position.
     * @param armDelay specifies the delay in seconds before moving arm.
     * @param armPos specifies the arm position.
     * @param wristDelay specifies the delay in seconds before moving wrist.
     * @param wristPos specifies the wrist position.
     */
    public void prepSubsystems(
        String owner, double elevatorDelay, double elevatorPos, double armDelay, double armPos, double wristDelay,
        double wirstPos)
    {
        prepSubsystems(owner, elevatorDelay, elevatorPos, armDelay, armPos, wristDelay, wirstPos, 0.0, null);
    }   //prepSubsystems

    /**
     * This method prep the subsystems for a certain operation.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystems.
     * @param elevatorPos specifies the elevator position.
     * @param armPos specifies the arm position.
     * @param wristPos specifies the wrist position.
     * @param timeout specifies the maximum time allowed for the operation.
     */
    public void prepSubsystems(String owner, double elevatorPos, double armPos, double wristPos, double timeout)
    {
        prepSubsystems(owner, 0.0, elevatorPos, 0.0, armPos, 0.0, wristPos, timeout, null);
    }   //prepSubsystems

    /**
     * This method prep the subsystems for a certain operation.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystems.
     * @param elevatorPos specifies the elevator position.
     * @param armPos specifies the arm position.
     * @param wristPos specifies the wrist position.
     */
    public void prepSubsystems(String owner, double elevatorPos, double armPos, double wristPos)
    {
        prepSubsystems(owner, 0.0, elevatorPos, 0.0, armPos, 0.0, wristPos, 0.0, null);
    }   //prepSubsystems

    public void prepForCubeGroundPickup(String owner, double delay, TrcEvent event)
    {
        prepSubsystems(
            owner, delay, RobotParams.ELEVATOR_CUBE_PICKUP_POSITION, delay, RobotParams.ARM_CUBE_PICKUP_POSITION,
            delay, RobotParams.WRIST_CUBE_PICKUP_POSITION, 0.0, event);
        ledIndicator.setScoreLevel(ObjectType.CUBE, scoreLevel);
        objType = ObjectType.CUBE;
    }   //prepForCubeGroundPickup

    public void prepForConeGroundPickup(String owner, double delay, TrcEvent event)
    {
        prepSubsystems(
            owner, delay, RobotParams.ELEVATOR_CONE_PICKUP_POSITION, delay, RobotParams.ARM_CONE_PICKUP_POSITION,
            delay, RobotParams.WRIST_CONE_PICKUP_POSITION, 0.0, event);
        ledIndicator.setScoreLevel(ObjectType.CONE, scoreLevel);
        objType = ObjectType.CONE;
    }   //prepForConeGroundPickup

    public void prepForConeSubstationPickup(String owner, double delay, TrcEvent event)
    {
        prepSubsystems(owner, delay, 5.0, delay, RobotParams.ARM_MIN_POS, delay, 5.0, 0.0, event);
        ledIndicator.setScoreLevel(ObjectType.CONE, scoreLevel);
        objType = ObjectType.CONE;
    }   //prepForConeSubstationPickup

    /**
     * This method configures the subsystems to Turtle Mode which means to retract everything so that the robot can
     * safely travel without damaging the subsystems.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystems.
     * @param event specifies the event to signal when the operation is completed.
     */
    public void turtleMode(String owner, TrcEvent event)
    {
        prepSubsystems(owner, 0.5, RobotParams.ELEVATOR_MIN_POS, 0.5, RobotParams.ARM_MIN_POS, 0.0, 7.0, 1.5, event);
    }   //turtleMode

}   //class Robot
