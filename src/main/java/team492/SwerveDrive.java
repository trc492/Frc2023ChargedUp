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

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.PrintStream;
import java.util.Scanner;
import java.util.stream.IntStream;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPidDrive;
import TrcCommonLib.trclib.TrcPurePursuitDrive;
import TrcCommonLib.trclib.TrcSwerveDriveBase;
import TrcCommonLib.trclib.TrcSwerveModule;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcRobot.RunMode;
import TrcFrcLib.frclib.FrcAnalogEncoder;
import TrcFrcLib.frclib.FrcCANCoder;
import TrcFrcLib.frclib.FrcCANFalcon;
import TrcFrcLib.frclib.FrcEncoder;
import TrcFrcLib.frclib.FrcFalconServo;
import TrcFrcLib.frclib.FrcPdp;

/**
 * This class creates the RobotDrive subsystem that consists of wheel motors and related objects for driving the
 * robot.
 */
public class SwerveDrive extends RobotDrive
{
    private static final boolean logPoseEvents = false;
    private static final boolean tracePidInfo = false;
    private static final String DBKEY_TEST_RUN_MOTORS = "Test/RunMotors";
    private static final String DBKEY_TEST_SET_ANGLE = "Test/SetAngle";
    private static final String DBKEY_TEST_SAVE_ANGLES = "Test/SaveAngles";
    private static final String DBKEY_TEST_ANGLE_TARGET = "Test/AngleTarget";
    private static final String DBKEY_TEST_SWERVE_ANGLES = "Test/SwerveAngles";
    //
    // Swerve steering motors and modules.
    //
    public final FrcEncoder lfEncoder, rfEncoder, lbEncoder, rbEncoder;
    public final FrcCANFalcon lfSteerMotor, rfSteerMotor, lbSteerMotor, rbSteerMotor;
    public final TrcSwerveModule lfWheel, lbWheel, rfWheel, rbWheel;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object.
     */
    public SwerveDrive(Robot robot)
    {
        super(robot);

        lfDriveMotor = createDriveMotor("lfDrive", RobotParams.CANID_LEFTFRONT_DRIVE, true);
        rfDriveMotor = createDriveMotor("rfDrive", RobotParams.CANID_RIGHTFRONT_DRIVE, true);
        lbDriveMotor = createDriveMotor("lbDrive", RobotParams.CANID_LEFTBACK_DRIVE, true);
        rbDriveMotor = createDriveMotor("rbDrive", RobotParams.CANID_RIGHTBACK_DRIVE, true);

        int[] zeros = getSteerZeroPositions();
        if (RobotParams.Preferences.useCANCoder)
        {
            lfEncoder = createCANCoder("lfEncoder", RobotParams.CANID_LEFTFRONT_STEER_ENCODER, true, zeros[0]);
            rfEncoder = createCANCoder("rfEncoder", RobotParams.CANID_RIGHTFRONT_STEER_ENCODER, true, zeros[1]);
            lbEncoder = createCANCoder("lbEncoder", RobotParams.CANID_LEFTBACK_STEER_ENCODER, true, zeros[2]);
            rbEncoder = createCANCoder("rbEncoder", RobotParams.CANID_RIGHTBACK_STEER_ENCODER, true, zeros[3]);
        }
        else
        {
            lfEncoder = createAnalogEncoder("lfEncoder", RobotParams.AIN_LEFTFRONT_STEER_ENCODER, true, zeros[0]);
            rfEncoder = createAnalogEncoder("rfEncoder", RobotParams.AIN_RIGHTFRONT_STEER_ENCODER, true, zeros[1]);
            lbEncoder = createAnalogEncoder("lbEncoder", RobotParams.AIN_LEFTBACK_STEER_ENCODER, true, zeros[2]);
            rbEncoder = createAnalogEncoder("rbEncoder", RobotParams.AIN_RIGHTBACK_STEER_ENCODER, true, zeros[3]);
        }

        lfSteerMotor = createSteerMotor("lfSteer", RobotParams.CANID_LEFTFRONT_STEER, false);
        TrcTimer.sleep(50);
        rfSteerMotor = createSteerMotor("rfSteer", RobotParams.CANID_RIGHTFRONT_STEER, false);
        TrcTimer.sleep(50);
        lbSteerMotor = createSteerMotor("lbSteer", RobotParams.CANID_LEFTBACK_STEER, false);
        TrcTimer.sleep(50);
        rbSteerMotor = createSteerMotor("rbSteer", RobotParams.CANID_RIGHTBACK_STEER, false);

        lfWheel = createSwerveModule("lfWheel", lfDriveMotor, lfSteerMotor, lfEncoder);
        rfWheel = createSwerveModule("rfWheel", rfDriveMotor, rfSteerMotor, rfEncoder);
        lbWheel = createSwerveModule("lbWheel", lbDriveMotor, lbSteerMotor, lbEncoder);
        rbWheel = createSwerveModule("rbWheel", rbDriveMotor, rbSteerMotor, rbEncoder);

        driveBase = new TrcSwerveDriveBase(
            lfWheel, lbWheel, rfWheel, rbWheel, gyro, RobotParams.ROBOT_DRIVE_WIDTH, RobotParams.ROBOT_DRIVE_LENGTH);
        driveBase.setSynchronizeOdometriesEnabled(false);
        driveBase.setOdometryScales(RobotParams.SWERVE_INCHES_PER_COUNT);

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
        // PID Coefficients for X and Y are the same for Swerve Drive.
        xPosPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.SWERVE_KP, RobotParams.SWERVE_KI, RobotParams.SWERVE_KD, RobotParams.SWERVE_KF);
        encoderXPidCtrl = new TrcPidController(
            "encoderXPidCtrl", xPosPidCoeff, RobotParams.SWERVE_TOLERANCE, driveBase::getXPosition);
        encoderXPidCtrl.setOutputLimit(RobotParams.DRIVE_MAX_XPID_POWER);
        encoderXPidCtrl.setRampRate(RobotParams.DRIVE_MAX_XPID_RAMP_RATE);
    
        yPosPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.SWERVE_KP, RobotParams.SWERVE_KI, RobotParams.SWERVE_KD, RobotParams.SWERVE_KF);
        encoderYPidCtrl = new TrcPidController(
            "encoderYPidCtrl", yPosPidCoeff, RobotParams.SWERVE_TOLERANCE, driveBase::getYPosition);
        encoderYPidCtrl.setOutputLimit(RobotParams.DRIVE_MAX_YPID_POWER);
        encoderYPidCtrl.setRampRate(RobotParams.DRIVE_MAX_YPID_RAMP_RATE);
    
        turnPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.GYRO_TURN_KP, RobotParams.GYRO_TURN_KI, RobotParams.GYRO_TURN_KD, RobotParams.GYRO_TURN_KF);
        gyroTurnPidCtrl = new TrcPidController(
            "gyroPidCtrl", turnPidCoeff, RobotParams.GYRO_TURN_TOLERANCE, driveBase::getHeading);
        gyroTurnPidCtrl.setOutputLimit(RobotParams.DRIVE_MAX_TURNPID_POWER);
        gyroTurnPidCtrl.setRampRate(RobotParams.DRIVE_MAX_TURNPID_RAMP_RATE);
        gyroTurnPidCtrl.setAbsoluteSetPoint(true);
    
        velPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.ROBOT_VEL_KP, RobotParams.ROBOT_VEL_KI, RobotParams.ROBOT_VEL_KD, RobotParams.ROBOT_VEL_KF);

        pidDrive = new TrcPidDrive("pidDrive", driveBase, encoderXPidCtrl, encoderYPidCtrl, gyroTurnPidCtrl);
        // AbsoluteTargetMode eliminates cumulative errors on multi-segment runs because drive base is keeping track
        // of the absolute target position.
        pidDrive.setAbsoluteTargetModeEnabled(true);
        pidDrive.setStallDetectionEnabled(true);
        pidDrive.setMsgTracer(robot.globalTracer, logPoseEvents, tracePidInfo);

        purePursuitDrive = new TrcPurePursuitDrive(
            "purePursuitDrive", driveBase, RobotParams.PPD_FOLLOWING_DISTANCE, RobotParams.PPD_POS_TOLERANCE,
            RobotParams.PPD_TURN_TOLERANCE, xPosPidCoeff, yPosPidCoeff, turnPidCoeff, velPidCoeff);
        purePursuitDrive.setMoveOutputLimit(RobotParams.PPD_MOVE_DEF_OUTPUT_LIMIT);
        purePursuitDrive.setRotOutputLimit(RobotParams.PPD_ROT_DEF_OUTPUT_LIMIT);
        purePursuitDrive.setFastModeEnabled(true);
        purePursuitDrive.setMsgTracer(robot.globalTracer, logPoseEvents, tracePidInfo);
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
        encoder.setScaleAndOffset(
            RobotParams.FALCON_CPR * RobotParams.STEER_GEAR_RATIO / RobotParams.CANCODER_CPR,
            steerZero);

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
        encoder.setScaleAndOffset(360.0, steerZero);
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
        errCode = steerMotor.motor.configFactoryDefault(10);
        if (errCode != ErrorCode.OK)
        {
            robot.globalTracer.traceWarn(
                funcName, "%s: Falcon.configFactoryDefault failed (code=%s).",
                name, errCode);
        }

        errCode = steerMotor.motor.configVoltageCompSaturation(RobotParams.BATTERY_NOMINAL_VOLTAGE, 10);
        if (errCode != ErrorCode.OK)
        {
            robot.globalTracer.traceWarn(
                funcName, "%s: Falcon.configVoltageCompSaturation failed (code=%s).",
                name, errCode);
        }

        steerMotor.motor.enableVoltageCompensation(true);

        steerMotor.setInverted(inverted);
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
        double encoderPos = steerEncoder.getPosition();
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
        TrcSwerveModule module = new TrcSwerveModule(name, driveMotor, servo);
        module.disableSteeringLimits();

        return module;
    }   //createSwerveModule

    /**
     * This method is called to set all swerve wheels to zero degrees without optimization.
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
        // setSteerAngleZero(false);
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
        setSteerAngleZero(false);
    }   //stopMode

    @Override
    public void startSteerCalibrate()
    {
        lfSteerMotor.set(0.0);
        rfSteerMotor.set(0.0);
        lbSteerMotor.set(0.0);
        rbSteerMotor.set(0.0);
        robot.dashboard.putBoolean(DBKEY_TEST_RUN_MOTORS, false);
        robot.dashboard.putBoolean(DBKEY_TEST_SET_ANGLE, false);
        robot.dashboard.putBoolean(DBKEY_TEST_SAVE_ANGLES, false);
    }   //startCalibrate

    @Override
    public void steerCalibratePeriodic()
    {
        if (robot.dashboard.getBoolean(DBKEY_TEST_SET_ANGLE, false))
        {
            ((TrcSwerveDriveBase) driveBase).setSteerAngle(
                robot.dashboard.getNumber(DBKEY_TEST_ANGLE_TARGET, 0), false);
            robot.dashboard.putBoolean(DBKEY_TEST_SET_ANGLE, false);
        }

        if (robot.dashboard.getBoolean(DBKEY_TEST_SAVE_ANGLES, false))
        {
            robot.dashboard.putBoolean(DBKEY_TEST_SAVE_ANGLES, false);
            saveSteerZeroPositions();
        }

        double power = robot.dashboard.getBoolean(DBKEY_TEST_RUN_MOTORS, false) ? RobotParams.STEER_CAL_POWER : 0.0;
        lfDriveMotor.set(power);
        rfDriveMotor.set(power);
        lbDriveMotor.set(power);
        rbDriveMotor.set(power);
        robot.dashboard.putString(
            DBKEY_TEST_SWERVE_ANGLES,
            String.format(
                "lf=%.2f/%.0f, rf=%.2f/%.0f, lr=%.2f/%.0f, rr=%.2f/%.0f",
                lfWheel.getSteerAngle(), lfSteerMotor.getMotorPosition(),
                rfWheel.getSteerAngle(), rfSteerMotor.getMotorPosition(),
                lbWheel.getSteerAngle(), lbSteerMotor.getMotorPosition(),
                rbWheel.getSteerAngle(), rbSteerMotor.getMotorPosition()));
    }   //calibratePeriodic

    /**
     * This method retrieves the steering zero calibration data from the calibration data file.
     *
     * @return calibration data of all four swerve modules.
     */
    private int[] getSteerZeroPositions()
    {
        final String funcName = "getSteerZeroPositions";

        try (Scanner in = new Scanner(new FileReader(RobotParams.TEAM_FOLDER + "/steerzeros.txt")))
        {
            return IntStream.range(0, 4).map(e -> in.nextInt()).toArray();
        }
        catch (Exception e)
        {
            robot.globalTracer.traceWarn(funcName, "Steer zero position file not found, using built-in defaults.");
            return RobotParams.STEER_ZEROS;
        }
    }   //getSteerZeroPositions

    /**
     * This method saves the steering zero calibration data to the calibration data file.
     */
    public void saveSteerZeroPositions()
    {
        final String funcName = "saveSteerZeroPositions";

        try (PrintStream out = new PrintStream(new FileOutputStream(RobotParams.TEAM_FOLDER + "/steerzeros.txt")))
        {
            out.printf("%.0f\n", TrcUtil.modulo(lfSteerMotor.getMotorPosition(), RobotParams.CANCODER_CPR));
            out.printf("%.0f\n", TrcUtil.modulo(rfSteerMotor.getMotorPosition(), RobotParams.CANCODER_CPR));
            out.printf("%.0f\n", TrcUtil.modulo(lbSteerMotor.getMotorPosition(), RobotParams.CANCODER_CPR));
            out.printf("%.0f\n", TrcUtil.modulo(rbSteerMotor.getMotorPosition(), RobotParams.CANCODER_CPR));
            robot.globalTracer.traceInfo(funcName, "Saved steer zeros!");
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
        if (owner == null || !enabled || driveBase.acquireExclusiveAccess(owner))
        {
            ((TrcSwerveDriveBase) driveBase).setAntiDefenseEnabled(owner, enabled);
            if (!enabled)
            {
                driveBase.releaseExclusiveAccess(owner);
            }
        }
    }   //setAntiDefenseEnabled

}   //class SwerveDrive
