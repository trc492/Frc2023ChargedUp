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

package team492.drivebases;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.PrintStream;
import java.util.Arrays;
import java.util.Scanner;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import TrcCommonLib.trclib.TrcTriggerThresholdZones;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPidDrive;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcPurePursuitDrive;
import TrcCommonLib.trclib.TrcSwerveDriveBase;
import TrcCommonLib.trclib.TrcSwerveModule;
// import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcRobot.RunMode;
import TrcFrcLib.frclib.FrcAnalogEncoder;
import TrcFrcLib.frclib.FrcCANCoder;
import TrcFrcLib.frclib.FrcCANFalcon;
// import TrcFrcLib.frclib.FrcDashboard;
import TrcFrcLib.frclib.FrcEncoder;
import TrcFrcLib.frclib.FrcFalconServo;
import TrcFrcLib.frclib.FrcPdp;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import team492.Robot;
import team492.RobotParams;

/**
 * This class creates the RobotDrive subsystem that consists of wheel motors and related objects for driving the
 * robot.
 */
public class SwerveDrive extends RobotDrive
{
    private static final String ZERO_CAL_FILE = "steerzeros.txt";
    private static final boolean logPoseEvents = false;
    private static final boolean tracePidInfo = false;
    // private static final FrcDashboard dashboard = FrcDashboard.getInstance();
    //
    // Swerve steering motors and modules.
    //
    public final FrcEncoder lfSteerEncoder, rfSteerEncoder, lbSteerEncoder, rbSteerEncoder;
    public final FrcEncoder[] steerEncoders;
    public final FrcCANFalcon lfSteerMotor, rfSteerMotor, lbSteerMotor, rbSteerMotor;
    public final TrcSwerveModule lfWheel, lbWheel, rfWheel, rbWheel;
    private final TrcTriggerThresholdZones tiltTrigger;
    private final TrcTriggerThresholdZones distanceTrigger;
    private String antiDefenseOwner = null;
    private Double startXPosition = null;
    private Double startYPosition = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object.
     */
    public SwerveDrive(Robot robot)
    {
        super(robot);

        lfDriveMotor = createDriveMotor("lfDrive", RobotParams.CANID_LEFTFRONT_DRIVE, false);
        rfDriveMotor = createDriveMotor("rfDrive", RobotParams.CANID_RIGHTFRONT_DRIVE, false);
        lbDriveMotor = createDriveMotor("lbDrive", RobotParams.CANID_LEFTBACK_DRIVE, false);
        rbDriveMotor = createDriveMotor("rbDrive", RobotParams.CANID_RIGHTBACK_DRIVE, false);

        double[] zeros = getSteerZeroPositions();
        if (RobotParams.Preferences.useCANCoder)
        {
            lfSteerEncoder = createCANCoder(
                "lfSteerEncoder", RobotParams.CANID_LEFTFRONT_STEER_ENCODER, true, zeros[0]);
            rfSteerEncoder = createCANCoder(
                "rfSteerEncoder", RobotParams.CANID_RIGHTFRONT_STEER_ENCODER, true, zeros[1]);
            lbSteerEncoder = createCANCoder(
                "lbSteerEncoder", RobotParams.CANID_LEFTBACK_STEER_ENCODER, true, zeros[2]);
            rbSteerEncoder = createCANCoder(
                "rbSteerEncoder", RobotParams.CANID_RIGHTBACK_STEER_ENCODER, true, zeros[3]);
        }
        else if (RobotParams.Preferences.useAnalogEncoder)
        {
            lfSteerEncoder = createAnalogEncoder(
                "lfSteerEncoder", RobotParams.AIN_LEFTFRONT_STEER_ENCODER, true, zeros[0]);
            rfSteerEncoder = createAnalogEncoder(
                "rfSteerEncoder", RobotParams.AIN_RIGHTFRONT_STEER_ENCODER, true, zeros[1]);
            lbSteerEncoder = createAnalogEncoder(
                "lbSteerEncoder", RobotParams.AIN_LEFTBACK_STEER_ENCODER, true, zeros[2]);
            rbSteerEncoder = createAnalogEncoder(
                "rbSteerEncoder", RobotParams.AIN_RIGHTBACK_STEER_ENCODER, true, zeros[3]);
        }
        else
        {
            throw new IllegalArgumentException("Must enable either useCANCoder or useAnalogEncoder.");
        }
        steerEncoders = new FrcEncoder[] {lfSteerEncoder, rfSteerEncoder, lbSteerEncoder, rbSteerEncoder};

        lfSteerMotor = createSteerMotor("lfSteer", RobotParams.CANID_LEFTFRONT_STEER, false);
        rfSteerMotor = createSteerMotor("rfSteer", RobotParams.CANID_RIGHTFRONT_STEER, false);
        lbSteerMotor = createSteerMotor("lbSteer", RobotParams.CANID_LEFTBACK_STEER, false);
        rbSteerMotor = createSteerMotor("rbSteer", RobotParams.CANID_RIGHTBACK_STEER, false);

        lfWheel = createSwerveModule("lfWheel", lfDriveMotor, lfSteerMotor, lfSteerEncoder);
        rfWheel = createSwerveModule("rfWheel", rfDriveMotor, rfSteerMotor, rfSteerEncoder);
        lbWheel = createSwerveModule("lbWheel", lbDriveMotor, lbSteerMotor, lbSteerEncoder);
        rbWheel = createSwerveModule("rbWheel", rbDriveMotor, rbSteerMotor, rbSteerEncoder);

        driveBase = new TrcSwerveDriveBase(
            lfWheel, lbWheel, rfWheel, rbWheel, gyro,
            RobotParams.ROBOT_WHEELBASE_WIDTH, RobotParams.ROBOT_WHEELBASE_LENGTH);
        driveBase.setSynchronizeOdometriesEnabled(false);
        driveBase.setOdometryScales(RobotParams.SWERVE_INCHES_PER_COUNT);

        if (RobotParams.Preferences.useAntiTipping)
        {
            driveBase.enableAntiTipping(
                new TrcPidController.PidParameters(
                    RobotParams.X_TIPPING_KP, RobotParams.X_TIPPING_KI, RobotParams.X_TIPPING_KD,
                    RobotParams.X_TIPPING_TOLERANCE, this::getGyroRoll),
                new TrcPidController.PidParameters(
                    RobotParams.Y_TIPPING_KP, RobotParams.Y_TIPPING_KI, RobotParams.Y_TIPPING_KD,
                    RobotParams.Y_TIPPING_TOLERANCE, this::getGyroPitch));
        }
        // if (RobotParams.Preferences.useExternalOdometry)
        // {
        //     //
        //     // Create the external odometry device that uses the right back encoder port as the X odometry and
        //     // the left and right front encoder ports as the Y1 and Y2 odometry. Gyro will serve as the angle
        //     // odometry.
        //     //
        //     TrcDriveBaseOdometry driveBaseOdometry = new TrcDriveBaseOdometry(
        //         new TrcDriveBaseOdometry.AxisSensor(rbDriveMotor, RobotParams.X_ODOMETRY_WHEEL_OFFSET),
        //         new TrcDriveBaseOdometry.AxisSensor[] {
        //             new TrcDriveBaseOdometry.AxisSensor(lfDriveMotor, RobotParams.Y_LEFT_ODOMETRY_WHEEL_OFFSET),
        //             new TrcDriveBaseOdometry.AxisSensor(rfDriveMotor, RobotParams.Y_RIGHT_ODOMETRY_WHEEL_OFFSET)},
        //         gyro);
        //     //
        //     // Set the drive base to use the external odometry device overriding the built-in one.
        //     //
        //     driveBase.setDriveBaseOdometry(driveBaseOdometry);
        //     driveBase.setOdometryScales(RobotParams.ODWHEEL_X_INCHES_PER_COUNT, RobotParams.ODWHEEL_Y_INCHES_PER_COUNT);
        // }
        // else
        // {
        //     driveBase.setOdometryScales(RobotParams.SWERVE_INCHES_PER_COUNT);
        // }

        if (robot.pdp != null)
        {
            robot.pdp.registerEnergyUsed(
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_LEFT_FRONT_DRIVE, "lfDriveMotor"),
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_LEFT_BACK_DRIVE, "lbDriveMotor"),
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_RIGHT_FRONT_DRIVE, "rfDriveMotor"),
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_RIGHT_BACK_DRIVE, "rbDriveMotor"),
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_LEFT_FRONT_STEER, "lfSteerMotor"),
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_LEFT_BACK_STEER, "lbSteerMotor"),
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_RIGHT_FRONT_STEER, "rfSteerMotor"),
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_RIGHT_BACK_STEER, "rbSteerMotor"));
        }
        //
        // Create and initialize PID controllers.
        //
        // PID Parameters for X and Y are the same for Swerve Drive.
        xPosPidCoeff = yPosPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.SWERVE_KP, RobotParams.SWERVE_KI, RobotParams.SWERVE_KD, RobotParams.SWERVE_KF);
        turnPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.GYRO_TURN_KP, RobotParams.GYRO_TURN_KI, RobotParams.GYRO_TURN_KD, RobotParams.GYRO_TURN_KF,
            RobotParams.GYRO_TURN_IZONE);
        velPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.ROBOT_VEL_KP, RobotParams.ROBOT_VEL_KI, RobotParams.ROBOT_VEL_KD, RobotParams.ROBOT_VEL_KF);

        TrcPidController.PidParameters xPosPidParams = new TrcPidController.PidParameters(
            xPosPidCoeff, RobotParams.SWERVE_TOLERANCE, driveBase::getXPosition);
        TrcPidController.PidParameters yPosPidParams = new TrcPidController.PidParameters(
            yPosPidCoeff, RobotParams.SWERVE_TOLERANCE, driveBase::getYPosition);
        TrcPidController.PidParameters turnPidParams = new TrcPidController.PidParameters(
            turnPidCoeff, RobotParams.GYRO_TURN_TOLERANCE, driveBase::getHeading);

        pidDrive = new TrcPidDrive(
            "pidDrive", driveBase, xPosPidParams, yPosPidParams, turnPidParams);

        pidDrive.getXPidCtrl().setOutputLimit(RobotParams.DRIVE_MAX_XPID_POWER);
        pidDrive.getXPidCtrl().setRampRate(RobotParams.DRIVE_MAX_XPID_RAMP_RATE);
        pidDrive.getYPidCtrl().setOutputLimit(RobotParams.DRIVE_MAX_YPID_POWER);
        pidDrive.getYPidCtrl().setRampRate(RobotParams.DRIVE_MAX_YPID_RAMP_RATE);
        pidDrive.getTurnPidCtrl().setOutputLimit(RobotParams.DRIVE_MAX_TURNPID_POWER);
        pidDrive.getTurnPidCtrl().setRampRate(RobotParams.DRIVE_MAX_TURNPID_RAMP_RATE);
        pidDrive.getTurnPidCtrl().setAbsoluteSetPoint(true);

        // AbsoluteTargetMode eliminates cumulative errors on multi-segment runs because drive base is keeping track
        // of the absolute target position.
        pidDrive.setAbsoluteTargetModeEnabled(true);
        pidDrive.setMsgTracer(robot.globalTracer, logPoseEvents, tracePidInfo);

        if (RobotParams.Preferences.useBalancePidDrive)
        {
            gyroPitchPidCoeff = new TrcPidController.PidCoefficients(
                RobotParams.GYRO_PITCH_KP, RobotParams.GYRO_PITCH_KI, RobotParams.GYRO_PITCH_KD,
                RobotParams.GYRO_PITCH_KF);
            TrcPidController.PidParameters gyroPitchPidParams = new TrcPidController.PidParameters(
                yPosPidCoeff, RobotParams.GYRO_PITCH_TOLERANCE, this::getGyroPitch, this::getBalanceDriveCompensation);

            balancePidDrive = new TrcPidDrive(
                "balancePidDrive", driveBase, xPosPidParams, gyroPitchPidParams, turnPidParams);

            balancePidDrive.getXPidCtrl().setAbsoluteSetPoint(true);
            balancePidDrive.getYPidCtrl().setAbsoluteSetPoint(true);
            balancePidDrive.getYPidCtrl().setOutputLimit(RobotParams.GYRO_PITCH_MAX_PID_POWER);
            balancePidDrive.getYPidCtrl().setRampRate(RobotParams.GYRO_PITCH_PID_RAMP_RATE);
            balancePidDrive.setMsgTracer(robot.globalTracer, logPoseEvents, tracePidInfo);
        }

        purePursuitDrive = new TrcPurePursuitDrive(
            "purePursuitDrive", driveBase, RobotParams.PPD_FOLLOWING_DISTANCE, RobotParams.PPD_POS_TOLERANCE,
            RobotParams.PPD_TURN_TOLERANCE, xPosPidCoeff, yPosPidCoeff, turnPidCoeff, velPidCoeff);
        purePursuitDrive.setMoveOutputLimit(RobotParams.PPD_MOVE_DEF_OUTPUT_LIMIT);
        purePursuitDrive.setRotOutputLimit(RobotParams.PPD_ROT_DEF_OUTPUT_LIMIT);
        purePursuitDrive.setFastModeEnabled(true);
        purePursuitDrive.setMsgTracer(robot.globalTracer, logPoseEvents, tracePidInfo);

        tiltTrigger = new TrcTriggerThresholdZones(
            "tiltTrigger", this::getGyroRoll, RobotParams.GYRO_TILT_THRESHOLDS, false);
        distanceTrigger = new TrcTriggerThresholdZones(
            "distanceTrigger", this::getDistanceTraveled, RobotParams.DRIVE_DISTANCE_THRESHOLDS, false);
    }   //SwerveDrive

    /**
     * This method creates an encoder for the steering motor.
     *
     * @param name specifies the instance name of the steering encoder.
     * @param encoderId specifies the CAN ID of the CANcoder.
     * @param inverted specifies true if the sensor direction should be inverted, false otherwise.
     * @param steerZero specifies the zero position.
     * @return the created steering encoder.
     */
    private FrcEncoder createCANCoder(String name, int encoderId, boolean inverted, double steerZero)
    {
        final String funcName = "createCANcoder";
        FrcEncoder encoder = new FrcCANCoder(name, encoderId);

        CANCoder canCoder = (CANCoder) encoder;
        ErrorCode errCode;
        // Reset encoder back to factory default to clear potential previous mis-configurations.
        errCode = canCoder.configFactoryDefault(10);
        if (errCode != ErrorCode.OK)
        {
            robot.globalTracer.traceWarn(
                funcName, "%s: CANcoder.configFactoryDefault failed (code=%s).",
                name, errCode);
        }
        errCode = canCoder.configFeedbackCoefficient(1.0, "cpr", SensorTimeBase.PerSecond, 10);
        if (errCode != ErrorCode.OK)
        {
            robot.globalTracer.traceWarn(
                funcName, "%s: CANcoder.configFeedbackCoefficient failed (code=%s).",
                name, errCode);
        }
        // Configure the encoder to initialize to absolute position value at boot.
        errCode = canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, 10);
        if (errCode != ErrorCode.OK)
        {
            robot.globalTracer.traceWarn(
                funcName, "%s: CANcoder.configSensorInitializationStrategy failed (code=%s).",
                name, errCode);
        }
        // Slow down the status frame rate to reduce CAN traffic.
        errCode = canCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100, 10);
        if (errCode != ErrorCode.OK)
        {
            robot.globalTracer.traceWarn(
                funcName, "%s: CANcoder.setStatusFramePeriod failed (code=%s).",
                name, errCode);
        }
        // Configure the sensor direction to match the steering motor direction.
        encoder.setInverted(inverted);
        // Normalize encoder to the range of 0 to 1.0 for a revolution (revolution per count).
        encoder.setScaleAndOffset(1.0 / RobotParams.CANCODER_CPR, steerZero);

        return encoder;
    }   //createCANCoder

    /**
     * This method creates an encoder for the steering motor.
     *
     * @param name specifies the instance name of the steering encoder.
     * @param encoderId specifies the analog channel of the analog encoder.
     * @param inverted specifies true if the sensor direction should be inverted, false otherwise.
     * @param steerZero specifies the zero position.
     * @return the created steering encoder.
     */
    private FrcEncoder createAnalogEncoder(String name, int encoderId, boolean inverted, double steerZero)
    {
        FrcEncoder encoder = new FrcAnalogEncoder(name, encoderId);

        encoder.setInverted(inverted);
        // Analog Encoder is already normalized to the range of 0 to 1.0 for a revolution (revolution per count).
        encoder.setScaleAndOffset(1.0, steerZero);
        return encoder;
    }   //createAnalogEncoder

    /**
     * This method creates a steering motor for a swerve module.
     *
     * @param name specifies the instance name of the steering motor.
     * @param motorCanID specifies the CAN ID of the steering motor.
     * @param inverted specifies true if the steering motor should be inverted, false otherwise.
     * @return the created steering motor.
     */
    private FrcCANFalcon createSteerMotor(String name, int motorCanID, boolean inverted)
    {
        final String funcName = "createSteerMotor";
        FrcCANFalcon steerMotor = new FrcCANFalcon(name, motorCanID);
        ErrorCode errCode;
        // Reset motor back to factory default to clear potential previous mis-configurations.
        errCode = steerMotor.motor.configFactoryDefault(30);
        if (errCode != ErrorCode.OK)
        {
            robot.globalTracer.traceWarn(
                funcName, "%s: Falcon.configFactoryDefault failed (code=%s).",
                name, errCode);
        }

        errCode = steerMotor.motor.configVoltageCompSaturation(RobotParams.BATTERY_NOMINAL_VOLTAGE, 30);
        if (errCode != ErrorCode.OK)
        {
            robot.globalTracer.traceWarn(
                funcName, "%s: Falcon.configVoltageCompSaturation failed (code=%s).",
                name, errCode);
        }

        steerMotor.motor.enableVoltageCompensation(true);

        steerMotor.setMotorInverted(inverted);
        steerMotor.setBrakeModeEnabled(true);

        return steerMotor;
    }   //createSteerMotor

    /**
     * This method creates the swerve module that consists of a driving motor and a steering motor.
     *
     * @param name specifies the swerve module instance name.
     * @param driveMotor specifies the drive motor object.
     * @param steerMotor specifies the steering motor object.
     * @param steerEncoder specifies the steering encoder object.
     * @return the created swerve module.
     */
    private TrcSwerveModule createSwerveModule(
        String name, FrcCANFalcon driveMotor, FrcCANFalcon steerMotor, FrcEncoder steerEncoder)
    {
        final String funcName = "createSwerveModule";
        // getPosition returns a value in the range of 0 to 1.0 of one revolution.

        // final int maxLoops = 10000;
        // double startTime = TrcTimer.getCurrentTime();
        // double firstReading = steerEncoder.getPosition();
        // double avg = firstReading;
        // for(int i = 1; i < maxLoops; i++)
        // {
        //     dashboard.putNumber("Graphs/" + name, steerEncoder.getPosition());
        //     avg += steerEncoder.getPosition();
        // }
        // robot.globalTracer.traceInfo(
        //     funcName, "%s: firstReading=%.0f, avgReading=%.0f, elapsedTime=%.3f",
        //     firstReading, avg/maxLoops, TrcTimer.getCurrentTime() - startTime);

        double encoderPos = steerEncoder.getPosition() * RobotParams.STEER_MOTOR_CPR;
        ErrorCode errCode = steerMotor.motor.setSelectedSensorPosition(encoderPos, 0, 10);
        if (errCode != ErrorCode.OK)
        {
            robot.globalTracer.traceWarn(
                funcName, "%s: Falcon.setSelectedSensorPosition failed (code=%s, pos=%.0f).",
                name, errCode, encoderPos);
        }

        // We have already synchronized the Falcon internal encoder with the zero adjusted absolute encoder, so
        // Falcon servo does not need to compensate for zero position.
        FrcFalconServo servo = new FrcFalconServo(
            name + ".servo", steerMotor, RobotParams.steerCoeffs, RobotParams.STEER_DEGREES_PER_COUNT, 0.0,
            RobotParams.STEER_MAX_REQ_VEL, RobotParams.STEER_MAX_ACCEL);
        servo.setControlMode(TalonFXControlMode.MotionMagic);
        servo.setPhysicalRange(0.0, 360.0);
        TrcSwerveModule module = new TrcSwerveModule(name, driveMotor, servo);
        module.disableSteeringLimits();

        return module;
    }   //createSwerveModule

    /**
     * This method is called to set all swerve wheels to zero degrees.
     *
     * @param optimize specifies true to optimize the shortest way to point the wheels forward, could end up at
     *        180-degree instead of zero, false to set wheel angle to absolute zero.
     */
    public void setSteerAngleZero(boolean optimize)
    {
        lfWheel.setSteerAngle(0.0, optimize);
        rfWheel.setSteerAngle(0.0, optimize);
        lbWheel.setSteerAngle(0.0, optimize);
        rbWheel.setSteerAngle(0.0, optimize);
    }   //setSteerAngleZero

    /**
     * This method is called to prepare the robot base before a robot mode is about to start.
     *
     * @param runMode specifies the current run mode.
     * @param prevMode specifies the previous run mode.
     */
    @Override
    public void startMode(RunMode runMode, RunMode prevMode)
    {
        super.startMode(runMode, prevMode);
        // Set all swerve steering pointing to absolute forward to start.
        setSteerAngleZero(false);
    }   //startMode

    /**
     * This method is called to prepare the robot base right after a robot mode has been stopped.
     *
     * @param runMode specifies the current run mode.
     * @param nextMode specifies the next run mode.
     */
    @Override
    public void stopMode(RunMode runMode, RunMode nextMode)
    {
        super.stopMode(runMode, nextMode);
        // setSteerAngleZero(false);
    }   //stopMode

    /**
     * This method retrieves the steering zero calibration data from the calibration data file.
     *
     * @return calibration data of all four swerve modules.
     */
    public static double[] getSteerZeroPositions()
    {
        final String funcName = "getSteerZeroPositions";

        try (Scanner in = new Scanner(new FileReader(RobotParams.TEAM_FOLDER + "/" + ZERO_CAL_FILE)))
        {
            double[] steerZeros = new double[4];

            for (int i = 0; i < steerZeros.length; i++)
            {
                steerZeros[i] = in.nextDouble();
            }

            return steerZeros;
        }
        catch (Exception e)
        {
            TrcDbgTrace.globalTraceWarn(funcName, "Steer zero position file not found, using built-in defaults.");
            return RobotParams.STEER_ZEROS;
        }
    }   //getSteerZeroPositions

    /**
     * This method saves the steering zero calibration data to the calibration data file.
     *
     * @param steerZeros specifies the steering zero calibration data to be saved.
     */
    public static void saveSteerZeroPositions(double[] steerZeros)
    {
        final String funcName = "saveSteerZeroPositions";

        try (PrintStream out = new PrintStream(new FileOutputStream(RobotParams.TEAM_FOLDER + "/" + ZERO_CAL_FILE)))
        {
            for (double zero : steerZeros)
            {
                out.printf("%f\n", zero);
            }
            TrcDbgTrace.globalTraceInfo(funcName, "Saved steer zeros: %s!", Arrays.toString(steerZeros));
        }
        catch (FileNotFoundException e)
        {
            e.printStackTrace();
        }
    }   //saveSteerZeroPositions

    /**
     * This method checks if anti-defense mode is enabled.
     *
     * @return true if anti-defense mode is enabled, false if disabled.
     */
    public boolean isAntiDefenseEnabled()
    {
        return ((TrcSwerveDriveBase) driveBase).isAntiDefenseEnabled();
    }   //isAntiDefenseEnabled

    /**
     * This method enables/disables the anti-defense mode where it puts all swerve wheels into an X-formation.
     * By doing so, it is very difficult for others to push us around.
     *
     * @param owner     specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                  ownership aware.
     * @param enabled   specifies true to enable anti-defense mode, false to disable.
     */
    public void setAntiDefenseEnabled(String owner, boolean enabled)
    {
        boolean requireOwnership = owner != null && enabled && !driveBase.hasOwnership(owner);

        if (requireOwnership && driveBase.acquireExclusiveAccess(owner))
        {
            antiDefenseOwner = owner;
        }

        if (!requireOwnership || antiDefenseOwner != null)
        {
            ((TrcSwerveDriveBase) driveBase).setAntiDefenseEnabled(owner, enabled);
            if (antiDefenseOwner != null)
            {
                driveBase.releaseExclusiveAccess(antiDefenseOwner);
                antiDefenseOwner = null;
            }
        }
    }   //setAntiDefenseEnabled

    /**
     * This method calculates the power compensation for balance PID drive.
     *
     * @param power specifies the drive power before compensation.
     * @return compensation drive power.
     */
    private double getBalanceDriveCompensation(double power)
    {
        // double angle = robot.robotDrive.getGyroYHeading();
        // if(Math.abs(angle) >= 10) {
        //     double comp = -0.005*Math.sin(Math.PI*angle/40);
        //     System.out.printf(">>> Angle:%.3f Comp: %.3f", angle, comp);
        //     return comp;
        // }
        return 0.0;
    }   //getBalanceDriveCompensation

    /**
     * This method returns the gyro pitch.
     *
     * @return gyro pitch.
     */
    public double getGyroPitch()
    {
        return gyro.getXHeading().value;
    }   //getGyroPitch

    /**
     * This method returns the gyro roll.
     * 
     * @return gyro roll.
     */
    public double getGyroRoll()
    {
        return gyro.getYHeading().value;
    }   //getGyroRoll

    /**
     * This method returns an adjusted absolute position by the robot's alliance.
     *
     * @param alliance specifies the robot alliance.
     * @param pos specifies the absolute position for the blue alliance.
     * @return returns unchanged pos if blue alliance, adjusted to the opposite side if red alliance.
     */
    public TrcPose2D adjustPosByAlliance(Alliance alliance, TrcPose2D pos)
    {
        if (alliance == Alliance.Red)
        {
            // no change on x, change y to the opposite side of the field.
            // 
            pos.y = RobotParams.FIELD_LENGTH - pos.y;
            pos.angle = (pos.angle + 180.0) % 360.0;
        }

        return pos;
    }   //adjustPosByAlliance

    /**
     * This method checks if the robot is in the balance zone.
     *
     * @return true if robot is in the balance zone.
     */
    public boolean inBalanceZone()
    {
        return tiltTrigger.getCurrentZone() == 2;
    }   //inBalanceZone

    public static enum TiltDir
    {
        TILT_LEFT,
        TILT_RIGHT
    }   //enum TiltDir

    /**
     * This method checks if the robot enters the balance zone.
     *
     * @return tilt direction before entering balance zone, null if not entering balance zone.
     */
    public TiltDir enteringBalanceZone()
    {
        TiltDir tiltDir = null;
        int prevZone = tiltTrigger.getPreviousZone();

        if ((prevZone == 1 || prevZone == 3) && tiltTrigger.getCurrentZone() == 2)
        {
            tiltDir = prevZone == 1? TiltDir.TILT_LEFT: TiltDir.TILT_RIGHT;
        }

        return tiltDir;
    }   //enteringBalanceZone

    /**
     * This method checks if the robot exits the balance zone.
     *
     * @return tilt direction after exiting balance zone, null if not exiting balance zone.
     */
    public TiltDir exitingBalanceZone()
    {
        TiltDir tiltDir = null;
        int currZone = tiltTrigger.getCurrentZone();

        if ((currZone == 1 || currZone == 3) && tiltTrigger.getPreviousZone() == 2)
        {
            tiltDir = currZone == 1? TiltDir.TILT_LEFT: TiltDir.TILT_RIGHT;
        }

        return tiltDir;
    }   //exitingBalanceZone

    /**
     * This method checks if the robot is about to level off where it is entering zone 1 or 3 from steeper tilt.
     *
     * @return true if starting to level off, false otherwise.
     */
    public boolean startingToLevel()
    {
        int currZone = tiltTrigger.getCurrentZone();
        return (currZone == 1 || currZone == 3) && tiltTrigger.getPreviousZone() != 2;
    }   //startingToLevel

    /**
     * This method enables the tilt sensor trigger.
     *
     * @param event specifies the event to signal when triggered.
     */
    public void enableTiltTrigger(TrcEvent event)
    {
        tiltTrigger.enableTrigger(event);
    }   //enableTiltTrigger

    /**
     * This method disables the tilt sensor trigger.
     */
    public void disableTiltTrigger()
    {
        tiltTrigger.disableTrigger();
    }   //disableTiltTrigger

    /**
     * This method returns the distance traveled since the distance trigger is enabled.
     *
     * @return distance traveled since the distance trigger is enabled.
     */
    private double getDistanceTraveled()
    {
        return startXPosition == null || startYPosition == null? 0.0:
               TrcUtil.magnitude(driveBase.getXPosition() - startXPosition,
                                 driveBase.getYPosition() - startYPosition);
    }   //getDistanceTraveled

    /**
     * This method enables the distance trigger and will trigger after the robot strafe the given distance.
     *
     * @param distance specifies the traveled distance for the trigger to activate.
     * @param event specifies the event to signal when triggered.
     */
    public void enableDistanceTrigger(double distance, TrcEvent event)
    {
        distanceTrigger.setThresholds(new double[]{distance});
        startXPosition = driveBase.getXPosition();
        startYPosition = driveBase.getYPosition();
        distanceTrigger.enableTrigger(event);
    }   //enableDistanceTrigger

    /**
     * This method disables the distance trigger.
     */
    public void disableDistanceTrigger()
    {
        distanceTrigger.disableTrigger();
    }   //disableDistanceTrigger

}   //class SwerveDrive
