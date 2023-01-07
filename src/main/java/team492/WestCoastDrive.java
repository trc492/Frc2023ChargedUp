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

import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPidDrive;
import TrcCommonLib.trclib.TrcPurePursuitDrive;
import TrcCommonLib.trclib.TrcSimpleDriveBase;
import TrcFrcLib.frclib.FrcPdp;

/**
 * This class creates the RobotDrive subsystem that consists of wheel motors and related objects for driving the
 * robot.
 */
public class WestCoastDrive extends RobotDrive
{
    private static final boolean logPoseEvents = false;
    private static final boolean tracePidInfo = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object.
     */
    public WestCoastDrive(Robot robot)
    {
        super(robot);

        lfDriveMotor = createDriveMotor("lfDriveMotor", RobotParams.CANID_LEFTFRONT_DRIVE, true);
        lbDriveMotor = createDriveMotor("lbDriveMotor", RobotParams.CANID_LEFTBACK_DRIVE, true);
        rfDriveMotor = createDriveMotor("rfDriveMotor", RobotParams.CANID_RIGHTFRONT_DRIVE, false);
        rbDriveMotor = createDriveMotor("rbDriveMotor", RobotParams.CANID_RIGHTBACK_DRIVE, false);

        lbDriveMotor.followMotor(lfDriveMotor);
        rbDriveMotor.followMotor(rfDriveMotor);

        driveBase = new TrcSimpleDriveBase(lfDriveMotor, rfDriveMotor, gyro);
        driveBase.setOdometryScales(RobotParams.WCD_INCHES_PER_COUNT);

        // if (RobotParams.Preferences.useExternalOdometry)
        // {
        //     //
        //     // Create the external odometry device that uses the left and right front encoder ports as the Y1 and Y2
        //     // odometry. Gyro will serve as the angle odometry.
        //     //
        //     TrcDriveBaseOdometry driveBaseOdometry = new TrcDriveBaseOdometry(
        //         new TrcDriveBaseOdometry.AxisSensor[] {
        //         new TrcDriveBaseOdometry.AxisSensor(lfDriveMotor, RobotParams.Y_LEFT_ODOMETRY_WHEEL_OFFSET),
        //         new TrcDriveBaseOdometry.AxisSensor(rfDriveMotor, RobotParams.Y_RIGHT_ODOMETRY_WHEEL_OFFSET)},
        //         gyro);
        //     //
        //     // Set the drive base to use the external odometry device overriding the built-in one.
        //     //
        //     driveBase.setDriveBaseOdometry(driveBaseOdometry);
        //     driveBase.setOdometryScales(RobotParams.ODWHEEL_Y_INCHES_PER_COUNT);
        // }
        // else
        // {
        //     driveBase.setOdometryScales(RobotParams.WCD_INCHES_PER_COUNT);
        // }

        if (robot.pdp != null)
        {
            robot.pdp.registerEnergyUsed(
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_LEFT_FRONT_DRIVE, "lfDriveMotor"),
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_LEFT_BACK_DRIVE, "lbDriveMotor"),
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_RIGHT_FRONT_DRIVE, "rfDriveMotor"),
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_RIGHT_BACK_DRIVE, "rbDriveMotor"));
        }

        //
        // Create and initialize PID controllers.
        //
        xPosPidCoeff = null;
        encoderXPidCtrl = null;

        yPosPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.WCD_KP, RobotParams.WCD_KI, RobotParams.WCD_KD, RobotParams.WCD_KF);
        encoderYPidCtrl = new TrcPidController(
            "encoderYPidCtrl", yPosPidCoeff, RobotParams.WCD_TOLERANCE, driveBase::getYPosition);
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

        pidDrive = new TrcPidDrive("pidDrive", driveBase, null, encoderYPidCtrl, gyroTurnPidCtrl);
        // AbsoluteTargetMode eliminates cumulative errors on multi-segment runs because drive base is keeping track
        // of the absolute target position.
        pidDrive.setAbsoluteTargetModeEnabled(true);
        pidDrive.setStallDetectionEnabled(true);
        pidDrive.setMsgTracer(robot.globalTracer, true, true);

        purePursuitDrive = new TrcPurePursuitDrive(
            "purePursuitDrive", driveBase, RobotParams.PPD_FOLLOWING_DISTANCE, RobotParams.PPD_POS_TOLERANCE,
            RobotParams.PPD_TURN_TOLERANCE, null, yPosPidCoeff, turnPidCoeff, velPidCoeff);
        purePursuitDrive.setMoveOutputLimit(RobotParams.PPD_MOVE_DEF_OUTPUT_LIMIT);
        purePursuitDrive.setStallDetectionEnabled(true);
        purePursuitDrive.setMsgTracer(robot.globalTracer, logPoseEvents, tracePidInfo);
    }   //WestCoastDrive

}   //class WestCoastDrive
