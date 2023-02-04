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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHEPIXYRWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team492;

import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * This class contains parameters and preferences related to all robot operations.
 */
public class RobotParams
{
    //
    // Robot preferences.
    //
    public static class Preferences
    {
        // Inputs
        public static final boolean useXboxController           = true;
        public static final boolean useButtonPanels             = false;
        public static final boolean doOneStickDrive             = true;
        // Sensors
        public static final boolean useNavX                     = false;
        public static final boolean usePdp                      = false;
        // Vision
        public static final boolean usePhotonVision             = false;
        public static final boolean useLimeLightVision          = false;
        public static final boolean useOpenCvVision             = true;
        public static final boolean useStreamCamera             = false;
        // Robot
        public static final boolean noRobot                     = true;
        // Drive Base
        public static final boolean useExternalOdometry         = false;
        public static final boolean swerveRobot                 = true;
        public static final boolean useVelocityControl          = false;
        public static final boolean useGyroAssist               = false;
        // Subsystems
        public static final boolean useSubsystems               = false;
        public static final boolean useGrabber                  = false;
        public static final boolean useVacuum                   = false;
        public static final boolean useLift                     = false;
        // Miscellaneous
        public static final boolean useTraceLog                 = true;
        public static final boolean doStatusUpdate              = true;
        public static final boolean showSubsystemStatus         = false;
        public static final boolean showVisionStatus            = false;
        // Debug
        public static final boolean debugPowerConsumption       = false;
        public static final boolean debugDriveBase              = false;
        public static final boolean debugPidDrive               = false;
        public static final boolean debugSubsystems             = false;
        public static final boolean debugVision                 = false;
        public static final boolean debugLoopTime               = false;
    }   //class Preferences

    public static final String ROBOT_NAME                       = "Generic_Robot";
    public static final String TEAM_FOLDER                      = "/home/lvuser/trc492";
    public static final double DASHBOARD_UPDATE_INTERVAL        = 0.1;          // in msec

    //
    // Field dimensions in inches.
    //
    public static final double FIELD_LENGTH                     = 54.0*12.0;
    public static final double FIELD_WIDTH                      = 27.0*12.0;

    //
    // Robot dimensions in inches.
    //
    public static final double ROBOT_WIDTH                      = 34.5;     // Frame dimensions, including bumpers.
    public static final double ROBOT_LENGTH                     = 37.0;     // Frame dimensions, including bumpers.

    public static final double ROBOT_DRIVE_WIDTH                = 23.25;    // Required by swerve drive base.
    public static final double ROBOT_DRIVE_LENGTH               = 25.625;   // Required by swerve drive base.

    //
    // Robot starting positions.
    //
    public static final TrcPose2D STARTPOS_1 = new TrcPose2D(  86.534,  -14.869,  -80.250);
    public static final TrcPose2D STARTPOS_2 = new TrcPose2D(  62.800,  -62.800,  -45.000);
    public static final TrcPose2D STARTPOS_3 = new TrcPose2D( -46.861,  -74.281,   32.250);
    public static final TrcPose2D[] startPos = {STARTPOS_1, STARTPOS_2, STARTPOS_3};

    //
    // Joystick ports.
    //
    public static final int JSPORT_DRIVER_LEFTSTICK             = 0;
    public static final int JSPORT_DRIVER_RIGHTSTICK            = 1;
    public static final int JSPORT_OPERATORSTICK                = 2;
    public static final int JSPORT_BUTTON_PANEL                 = 3;
    public static final int JSPORT_SWITCH_PANEL                 = 4;
    public static final int XBOX_DRIVERCONTROLLER               = 5;

    //
    // CAN IDs.
    //
    public static final int CANID_LEFTFRONT_DRIVE               = 3;    // Orange: 40A
    public static final int CANID_RIGHTFRONT_DRIVE              = 4;    // Yellow: 40A
    public static final int CANID_LEFTBACK_DRIVE                = 5;    // Green: 40A
    public static final int CANID_RIGHTBACK_DRIVE               = 6;    // Blue: 40A
    public static final int CANID_INTAKE_LEFT                   = 27; 
    public static final int CANID_INTAKE_RIGHT                  = 28;  
    public static final int CANID_LIFT                          = 420; //random number (must find value)

    // Applicable only for Swerve Drive.
    public static final int CANID_LEFTFRONT_STEER_ENCODER       = 7;    // Orange
    public static final int CANID_RIGHTFRONT_STEER_ENCODER      = 8;    // Yellow
    public static final int CANID_LEFTBACK_STEER_ENCODER        = 9;    // Green
    public static final int CANID_RIGHTBACK_STEER_ENCODER       = 10;   // Blue

    public static final int CANID_LEFTFRONT_STEER               = 13;   // Orange: 40A
    public static final int CANID_RIGHTFRONT_STEER              = 14;   // Yellow: 40A
    public static final int CANID_LEFTBACK_STEER                = 15;   // Green: 40A
    public static final int CANID_RIGHTBACK_STEER               = 16;   // Blue: 40A

    public static final int CANID_PCM                           = 30;
    public static final int CANID_PDP                           = 31;

    //
    // PDP Channels.
    //
    public static final int PDP_CHANNEL_LEFT_FRONT_DRIVE        = 11;   // Orange: 40A
    public static final int PDP_CHANNEL_RIGHT_FRONT_DRIVE       = 5;    // Yellow: 40A
    public static final int PDP_CHANNEL_LEFT_BACK_DRIVE         = 13;   // Green: 40A
    public static final int PDP_CHANNEL_RIGHT_BACK_DRIVE        = 3;    // Blue: 40A
    public static final int PDP_CHANNEL_LEFT_FRONT_STEER        = 10;   // Orange: 40A
    public static final int PDP_CHANNEL_RIGHT_FRONT_STEER       = 6;    // Yellow: 40A
    public static final int PDP_CHANNEL_LEFT_BACK_STEER         = 12;   // Green: 40A
    public static final int PDP_CHANNEL_RIGHT_BACK_STEER        = 4;    // Blue: 40A

    public static final double BATTERY_NOMINAL_VOLTAGE          = 12.0;
    public static final double BATTERY_CAPACITY_WATT_HOUR       = 18.0*12.0;

    //
    // Analog Input ports.
    //
    public static final int AIN_PRESSURE_SENSOR                 = 0;

    //
    // Digital Input/Output ports.
    //
    public static final int DIO_LIFT_LOWER_LIMIT_SWITCH        = 5; //random number




    //
    // PWM channels.
    //
    public static final int PWM_CHANNEL_LED                     = 0;
    public static final int VACUUM                              = 1;
    public static final int NUM_LEDS                            = 60;

    //
    // Relay channels.
    //

    //
    // Pneumatic channels.
    //
    public static final int PNEUMATIC_LEFT_GRABBER_RETRACT      = 0;
    public static final int PNEUMATIC_LEFT_GRABBER_EXTEND       = 1;
    public static final int PNEUMATIC_RIGHT_GRABBER_RETRACT     = 2;
    public static final int PNEUMATIC_RIGHT_GRABBER_EXTEND      = 3;

    public static final int PNEUMATIC_INTAKE_EXTEND             = 4; 
    public static final int PNEUMATIC_INTAKE_RETRACT            = 5; 

    public static final int PNEUMATIC_LIFT_RETRACT              = 422; //random number
    public static final int PNEUMATIC_LIFT_EXTEND               = 423; //random number




    //
    // Ultrasonic sensors.
    //
    // public static final double SONAR_INCHES_PER_VOLT            = 1.0/0.0098; //9.8mV per inch
    // public static final double SONAR_ERROR_THRESHOLD            = 50.0; //value should not jump 50-in per time slice.

    //
    // Vision subsystem.
    //
    public static final int CAMERA_IMAGE_WIDTH                  = 160;
    public static final int CAMERA_IMAGE_HEIGHT                 = 120;
    public static final double CAMERA_DATA_TIMEOUT              = 0.5;  // 500ms
    public static final double VISION_TARGET_HEIGHT             = 104.0;// Inches from the floor
    public static final double CAMERA_Y_OFFSET                  = 0.0;  // Inches from the center of the robot
    public static final double CAMERA_X_OFFSET                  = 0.0;  // Inches from the center of the robot
    public static final double CAMERA_HEIGHT                    = 0.0;  // Inches from the floor
    public static final double CAMERA_ANGLE                     = 0.0;  // Degrees from horizontal
    public static final Transform3d CAMERA_TRANSFORM3D          = new Transform3d(
        new Translation3d(CAMERA_X_OFFSET, CAMERA_Y_OFFSET, CAMERA_HEIGHT),
        new Rotation3d(0.0, CAMERA_ANGLE, 0.0));
    public static final double APRILTAG_SIZE                    = 6.0 / TrcUtil.INCHES_PER_METER;   //  in meters
    public static final double WEBCAM_FX                        = 821.993;  // in pixels
    public static final double WEBCAM_FY                        = 821.993;  // in pixels
    public static final double WEBCAM_CX                        = 330.489;  // in pixels
    public static final double WEBCAM_CY                        = 248.997;  // in pixels

    //
    // DriveBase subsystem.
    //
    public static final double WCD_INCHES_PER_COUNT             = 2.2421;
    public static final double WCD_KP                           = 0.011;
    public static final double WCD_KI                           = 0.0;
    public static final double WCD_KD                           = 0.0013;
    public static final double WCD_KF                           = 0.0;
    public static final double WCD_TOLERANCE                    = 2.0;

    public static final double MECANUM_X_INCHES_PER_COUNT       = 2.2421;
    public static final double MECANUM_X_KP                     = 0.011;
    public static final double MECANUM_X_KI                     = 0.0;
    public static final double MECANUM_X_KD                     = 0.0013;
    public static final double MECANUM_X_KF                     = 0.0;
    public static final double MECANUM_X_TOLERANCE              = 2.0;

    public static final double MECANUM_Y_INCHES_PER_COUNT       = 2.2421;
    public static final double MECANUM_Y_KP                     = 0.011;
    public static final double MECANUM_Y_KI                     = 0.0;
    public static final double MECANUM_Y_KD                     = 0.0013;
    public static final double MECANUM_Y_KF                     = 0.0;
    public static final double MECANUM_Y_TOLERANCE              = 2.0;

    public static final double SWERVE_INCHES_PER_COUNT          = 9.072106867127145344367826764411e-4;
    public static final double SWERVE_KP                        = 0.02;
    public static final double SWERVE_KI                        = 0.0;
    public static final double SWERVE_KD                        = 0.0;
    public static final double SWERVE_KF                        = 0.0;
    public static final double SWERVE_TOLERANCE                 = 2.0;

    public static final double GYRO_TURN_KP                     = 0.012;
    public static final double GYRO_TURN_KI                     = 0.0;
    public static final double GYRO_TURN_KD                     = 0.0;
    public static final double GYRO_TURN_KF                     = 0.0;
    public static final double GYRO_TURN_TOLERANCE              = 2.0;

    public static final double GYRO_ASSIST_TURN_GAIN            = 0.1;

    public static final double ROBOT_MAX_VELOCITY               = 180.0;
    public static final double ROBOT_MAX_ACCELERATION           = 2100.0;
    public static final double ROBOT_MAX_TURN_RATE              = 1000.0;
    public static final double ROBOT_VEL_KP                     = 0.0;
    public static final double ROBOT_VEL_KI                     = 0.0;
    public static final double ROBOT_VEL_KD                     = 0.0;
    // KF should be set to the reciprocal of max tangential velocity (time to travel unit distance), units: sec./in.
    public static final double ROBOT_VEL_KF                     = 1.0 / ROBOT_MAX_VELOCITY;

    public static final double DRIVE_SLOW_SCALE                 = 0.5;
    public static final double TURN_SLOW_SCALE                  = 0.3;
    public static final double DRIVE_MEDIUM_SCALE               = 0.75;
    public static final double TURN_MEDIUM_SCALE                = 0.6;
    public static final double DRIVE_FAST_SCALE                 = 1.0;
    public static final double TURN_FAST_SCALE                  = 0.8;

    public static final double DRIVE_MAX_XPID_POWER             = 0.5;
    public static final double DRIVE_MAX_YPID_POWER             = 0.6;
    public static final double DRIVE_MAX_TURNPID_POWER          = 1.0;

    public static final double DRIVE_MAX_XPID_RAMP_RATE         = 0.5;
    public static final double DRIVE_MAX_YPID_RAMP_RATE         = 0.6;
    public static final double DRIVE_MAX_TURNPID_RAMP_RATE      = 1.0;

    public static final double DRIVE_RAMP_RATE                  = 0.2;

    // Applicable only for Swerve Drive.
    public static final double CANCODER_CPR                     = 4096.0;
    public static final double FALCON_CPR                       = 2048.0;
    public static final double STEER_ENCODER_SCALE              = FALCON_CPR / CANCODER_CPR;
    public static final double STEER_GEAR_RATIO                 = (24.0/12.0) * (72.0/14.0);

    public static final double STEER_DEGREES_PER_TICK           = 360.0 / CANCODER_CPR;
    public static final double STEER_DEGREES_PER_COUNT          = 360.0 / (FALCON_CPR*STEER_GEAR_RATIO);
    public static final double STEER_MAX_REQ_VEL                = 1000.0;   // deg/sec. max commanded velocity, not necessarily max vel
    public static final double STEER_MAX_ACCEL                  = 5000.0;   // deg/sec^2
    // ((theoretical max rpm * speed loss constant / gear ratio) / 60 sec/min) * 360 deg/rev
    public static final double STEER_MAX_VEL                    = ((18700.0 * 0.81 / 56.67) / 60.0) * 360.0;        // deg/sec
    public static final double STEER_MAX_VEL_COUNT_PER_100MS    = (STEER_MAX_VEL / STEER_DEGREES_PER_COUNT) / 10.0; // count/100ms

    // order is lf, rf, lr, rr
    public static final int[] STEER_ZEROS                       = new int[] {2167, 3756, 1194, 3485};   // this is a backup if the zeros file isn't found

    public static final TrcPidController.PidCoefficients magicSteerCoeff =
        new TrcPidController.PidCoefficients(2.0, 0.01, 0.0, 1023.0 / STEER_MAX_VEL_COUNT_PER_100MS, 5.0 / STEER_DEGREES_PER_COUNT);
    public static final double STEER_KP                         = 1.1;
    public static final double STEER_KI                         = 0.0;
    public static final double STEER_KD                         = 14.0;
    public static final double STEER_KF                         = 0.0;
    public static final double STEER_CAL_POWER                  = 0.1;
    public static final TrcPidController.PidCoefficients steerCoeffs =
        new TrcPidController.PidCoefficients(STEER_KP, STEER_KI, STEER_KD, STEER_KF);

    public static final double PPD_FOLLOWING_DISTANCE           = 12.0;
    public static final double PPD_POS_TOLERANCE                = 2.0;
    public static final double PPD_TURN_TOLERANCE               = 2.0;
    public static final double PPD_MOVE_DEF_OUTPUT_LIMIT        = 0.5;
    public static final double PPD_ROT_DEF_OUTPUT_LIMIT         = 0.3;

    //
    // Lift subsystem
    //
    public static final double LIFT_KP                       = 0.2;      //0.06;
    public static final double LIFT_KI                       = 0.0;
    public static final double LIFT_KD                       = 0.0;      //0.005;
    public static final double LIFT_TOLERANCE                = 1.0;
    public static final int LIFT_ENCODER_PPR                 = 4096;
    public static final double LIFT_INCHES_PER_COUNT         = 1.392027924751009e-4;
    public static final double LIFT_OFFSET                   = 29.6785;
    public static final double LIFT_CAL_POWER                = 0.5;
    public static final boolean LIFT_MOTOR_INVERTED          = true;
    public static final double LIFT_MIN_POS                  = 20.0;
    public static final double LIFT_MAX_POS                  = 65.0;
    public static final double LIFT_RAISED                   = 23.3; //random number
    public static final double LIFT_LOWERED                  = 24.3; //random number



    //
    // Other subsystems.
    //

}   //class RobotParams
