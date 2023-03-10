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
        public static final boolean useDriverXboxController     = true;
        public static final boolean useOperatorXboxController   = false;
        public static final boolean useButtonPanels             = true;
        public static final boolean doOneStickDrive             = false;
        // Sensors
        public static final boolean useNavX                     = true;
        public static final boolean usePdp                      = false;
        // public static final boolean usePressureSensor           = false;
        // Vision
        public static final boolean usePhotonVision             = true;
        public static final boolean useLimeLightVision          = false;
        public static final boolean useOpenCvVision             = false;
        public static final boolean useStreamCamera             = false;
        // Robot
        public static final boolean noRobot                     = false;
        // Drive Base
        public static final boolean useExternalOdometry         = false;
        public static final boolean useCANCoder                 = false;
        public static final boolean useAnalogEncoder            = true;
        public static final boolean useVelocityControl          = false;
        public static final boolean useGyroAssist               = false;
        public static final boolean useAntiTipping              = false;
        public static final boolean useBalancePidDrive          = false;
        // Subsystems
        public static final boolean useSubsystems               = true;
        public static final boolean useElevator                 = true;
        public static final boolean useArm                      = true;
        public static final boolean useGrabber                  = true;
        public static final boolean useVacuum                   = false;
        public static final boolean useIntake                   = true;
        // Miscellaneous
        public static final boolean useTraceLog                 = true;
        public static final boolean doStatusUpdate              = true;
        // Debug
        public static final boolean debugPowerConsumption       = false;

        public static final boolean debugVision                 = false;
        public static final boolean debugPhoton                 = false;
        public static final boolean debugLimeLight              = false;
        public static final boolean debugOpenCv                 = false;

        public static final boolean debugDriveBase              = false;
        public static final boolean debugPurePursuitDrive       = false;
        public static final boolean debugPidDrive               = false;

        public static final boolean debugSubsystems             = true;
        public static final boolean debugSwerveSteering         = false;
        public static final boolean debugArmEncoder             = false;
        public static final boolean debugLoopTime               = false;
    }   //class Preferences

    public static final String ROBOT_NAME                       = "Robot2023_ChargedUp";
    public static final String TEAM_FOLDER                      = "/home/lvuser/trc492";
    public static final double DASHBOARD_UPDATE_INTERVAL        = 0.1;          // in msec
    //
    // Field dimensions in inches.
    //
    public static final double FIELD_LENGTH                     = 54.0*12.0 + 3.25;
    public static final double FIELD_WIDTH                      = 26.0*12.0 + 3.5;
    //
    // Robot dimensions in inches.
    //
    public static final double ROBOT_WIDTH                      = (27.0 + 6.0); // Frame dimensions, including bumpers.
    public static final double ROBOT_LENGTH                     = (30.5 + 6.0); // Frame dimensions, including bumpers.

    public static final double ROBOT_WHEELBASE_WIDTH            = 23.25;        // Required by swerve drive base.
    public static final double ROBOT_WHEELBASE_LENGTH           = 22.44;        // Required by swerve drive base.
    //
    // Robot starting positions. All dimensions are from Game Manual, do not adjust.
    //
    public static final double GRID_TAPE_EDGE_BLUE_Y            = 54.25;
    public static final double STARTPOS_Y_OFFSET                = 24.0;
    public static final double STARTPOS_BLUE_Y                  = GRID_TAPE_EDGE_BLUE_Y +
                                                                  RobotParams.ROBOT_LENGTH/2.0 +
                                                                  STARTPOS_Y_OFFSET;
    public static final double STARTPOS_RED_Y                   = FIELD_LENGTH - STARTPOS_BLUE_Y;
    public static final double STARTPOS_1_X                     = -42.19;
    public static final double STARTPOS_2_X                     = -108.19;
    public static final double STARTPOS_3_X                     = -174.19;
    public static final double[] startPosX                      = {STARTPOS_1_X, STARTPOS_2_X, STARTPOS_3_X};
    public static final TrcPose2D STARTPOS_BLUE_1 = new TrcPose2D(STARTPOS_1_X, STARTPOS_BLUE_Y, 180.0);
    public static final TrcPose2D STARTPOS_BLUE_2 = new TrcPose2D(STARTPOS_2_X, STARTPOS_BLUE_Y, 180.0);
    public static final TrcPose2D STARTPOS_BLUE_3 = new TrcPose2D(STARTPOS_3_X, STARTPOS_BLUE_Y, 180.0);
    public static final TrcPose2D STARTPOS_RED_1 = new TrcPose2D(STARTPOS_1_X, STARTPOS_RED_Y, 0.0);
    public static final TrcPose2D STARTPOS_RED_2 = new TrcPose2D(STARTPOS_2_X, STARTPOS_RED_Y, 0.0);
    public static final TrcPose2D STARTPOS_RED_3 = new TrcPose2D(STARTPOS_3_X, STARTPOS_RED_Y, 0.0);
    public static final TrcPose2D[][] startPos =
    {
        {STARTPOS_BLUE_1, STARTPOS_BLUE_2, STARTPOS_BLUE_3},
        {STARTPOS_RED_1, STARTPOS_RED_2, STARTPOS_RED_3}
    };
    //
    // Game element locations and dimensions.
    //
    public static final double GAME_PIECE_1_X                   = -36.25;
    public static final double GAME_PIECE_2_X                   = GAME_PIECE_1_X - 48.0;
    public static final double GAME_PIECE_3_X                   = GAME_PIECE_2_X - 48.0;
    public static final double GAME_PIECE_4_X                   = GAME_PIECE_3_X - 48.0;
    public static final double GAME_PIECE_BLUE_Y                = GRID_TAPE_EDGE_BLUE_Y + 18*12.0 + 8.0;
    public static final double GAME_PIECE_RED_Y                 = FIELD_LENGTH - GAME_PIECE_BLUE_Y;

    public static final double CHARGING_STATION_WIDTH           = 8.0*12.0 + 1.25;
    public static final double CHARGING_STATION_DEPTH           = 6.0*12.0 + 4.125;
    public static final double CHARGING_STATION_CENTER_X        = -9.0*12.0;
    public static final double CHARGING_STATION_CENTER_BLUE_Y   = GRID_TAPE_EDGE_BLUE_Y + 8.0*12.0 + 2.625;
    public static final double CHARGING_STATION_CENTER_RED_Y    = FIELD_LENGTH - CHARGING_STATION_CENTER_BLUE_Y;

    public static final double CENTER_BETWEEN_CHARGING_STATION_AND_FIELD_EDGE_X =
        (RobotParams.CHARGING_STATION_CENTER_X + RobotParams.CHARGING_STATION_WIDTH/2.0)/2.0;

    public static final double HIGH_POLE_HEIGHT                 = 3.0*12.0 + 10.0;
    public static final double LOW_POLE_HEIGHT                  = 2.0*12.0 + 10.0;
    public static final double HIGH_POLE_TAPE_HEIGHT            = HIGH_POLE_HEIGHT - 2.1875;
    public static final double LOW_POLE_TAPE_HEIGHT             = LOW_POLE_HEIGHT - 10.0;
    public static final double CONE_HALF_HEIGHT                 = 6.34375;  // Inches
    public static final double CONE_HALF_WIDTH                  = 4.0;      // Inches
    public static final double CUBE_HALF_HEIGHT                 = 4.5;      // Inches

    //
    // Joystick ports.
    //
    public static final int XBOX_DRIVER_CONTROLLER              = 0;
    public static final int JSPORT_DRIVER_LEFTSTICK             = 0;
    public static final int JSPORT_DRIVER_RIGHTSTICK            = 1;
    public static final int XBOX_OPERATOR_CONTROLLER            = 2;
    public static final int JSPORT_OPERATORSTICK                = 2;
    public static final int JSPORT_BUTTON_PANEL                 = 3;
    public static final int JSPORT_SWITCH_PANEL                 = 4;
    //
    // CAN IDs.
    //
    public static final int CANID_LEFTFRONT_DRIVE               = 3;
    public static final int CANID_RIGHTFRONT_DRIVE              = 4;
    public static final int CANID_LEFTBACK_DRIVE                = 5;
    public static final int CANID_RIGHTBACK_DRIVE               = 6;

    public static final int CANID_ELEVATOR                      = 7;
    public static final int CANID_ARM                           = 8;
    public static final int CANID_INTAKE_LEFT                   = 17;
    public static final int CANID_INTAKE_RIGHT                  = 18;
    // Applicable only for Swerve Drive.
    public static final int CANID_LEFTFRONT_STEER               = 13;
    public static final int CANID_RIGHTFRONT_STEER              = 14;
    public static final int CANID_LEFTBACK_STEER                = 15;
    public static final int CANID_RIGHTBACK_STEER               = 16;

    public static final int CANID_LEFTFRONT_STEER_ENCODER       = 23;
    public static final int CANID_RIGHTFRONT_STEER_ENCODER      = 24;
    public static final int CANID_LEFTBACK_STEER_ENCODER        = 25;
    public static final int CANID_RIGHTBACK_STEER_ENCODER       = 26;

    public static final int CANID_PCM                           = 30;
    public static final int CANID_PDP                           = 31;
    //
    // PDP Channels.
    //
    public static final int PDP_CHANNEL_LEFT_FRONT_DRIVE        = 17;   // Orange: 40A
    public static final int PDP_CHANNEL_RIGHT_FRONT_DRIVE       = 12;   // Yellow: 40A
    public static final int PDP_CHANNEL_LEFT_BACK_DRIVE         = 4;    // Green: 40A
    public static final int PDP_CHANNEL_RIGHT_BACK_DRIVE        = 8;    // Blue: 40A

    public static final int PDP_CHANNEL_LEFT_FRONT_STEER        = 16;   // Orange: 40A
    public static final int PDP_CHANNEL_RIGHT_FRONT_STEER       = 13;   // Yellow: 40A
    public static final int PDP_CHANNEL_LEFT_BACK_STEER         = 3;    // Green: 40A
    public static final int PDP_CHANNEL_RIGHT_BACK_STEER        = 9;    // Blue: 40A

    public static final int PDP_CHANNEL_ELEVATOR                = 2;    // Purple: 40A
    public static final int PDP_CHANNEL_ARM                     = 1;    // Gray: 40A
    public static final int PDP_CHANNEL_INTAKE_LEFT             = 15;   // Purple: 30A
    public static final int PDP_CHANNEL_INTAKE_RIGHT            = 14;   // Gray: 30A

    public static final int PDP_CHANNEL_ROBORIO                 = 20;   // 10A
    public static final int PDP_CHANNEL_VRM                     = 22;   // 10A
    public static final int PDP_CHANNEL_PCM                     = 19;
    public static final int PDP_CHANNEL_RADIO_POE               = 18;   // 10A
    public static final int PDP_CHANNEL_ETHERNET_SWITCH         = 21;
    public static final int PDP_CHANNEL_LIMELIGHT               = 0;
    public static final int PDP_CHANNEL_LED                     = 10;
    public static final int PDP_DIGITAL_SENSORS                 = 11;

    public static final double BATTERY_NOMINAL_VOLTAGE          = 12.0;
    public static final double BATTERY_CAPACITY_WATT_HOUR       = 18.0*12.0;
    //
    // Analog Input ports.
    //
    // public static final int AIN_PRESSURE_SENSOR                 = 0;
    public static final int AIN_LEFTFRONT_STEER_ENCODER         = 0;    // Black
    public static final int AIN_RIGHTFRONT_STEER_ENCODER        = 1;    // Brown
    public static final int AIN_LEFTBACK_STEER_ENCODER          = 2;    // Red
    public static final int AIN_RIGHTBACK_STEER_ENCODER         = 3;    // Orange
    //
    // Digital Input/Output ports.
    //
    public static final int DIO_INTAKE_SENSOR                   = 0;    // Black
    public static final int DIO_GRABBER_SENSOR                  = 1;
    //
    // PWM channels.
    //
    public static final int NUM_LEDS                            = 95;
    public static final int PWM_CHANNEL_LED                     = 9;
    public static final int PWM_VACUUM                          = 0;
    //
    // Relay channels.
    //

    //
    // Pneumatic channels.
    //
    public static final int PNEUMATIC_CONE_GRABBER_RETRACT      = 6;
    public static final int PNEUMATIC_CONE_GRABBER_EXTEND       = 1;
    public static final int PNEUMATIC_CUBE_GRABBER_RETRACT      = 2;
    public static final int PNEUMATIC_CUBE_GRABBER_EXTEND       = 5;
    public static final int PNEUMATIC_INTAKE_RETRACT            = 4;
    public static final int PNEUMATIC_INTAKE_EXTEND             = 3;
    //
    // Ultrasonic sensors.
    //
    // public static final double SONAR_INCHES_PER_VOLT            = 1.0/0.0098; //9.8mV per inch
    // public static final double SONAR_ERROR_THRESHOLD            = 50.0; //value should not jump 50-in per time slice.

    //
    // Vision subsystem.
    //
    public static final int CAMERA_IMAGE_WIDTH                  = 320;
    public static final int CAMERA_IMAGE_HEIGHT                 = 240;
    public static final double CAMERA_DATA_TIMEOUT              = 0.5;      // 500ms
    public static final double VISION_TIMEOUT                   = 0.5;      // 500ms
    public static final double VISION_TARGET_HEIGHT             = 104.0;    // Inches from the floor
    public static final double CAMERA_Y_OFFSET                  = 11.6625;  // Inches from the center of the robot
    public static final double CAMERA_X_OFFSET                  = 0.1875;   // 3/16 Inches from the center of the robot
    public static final double CAMERA_HEIGHT                    = 41.7;     // Inches from the floor
    public static final double CAMERA_PITCH                     = -44.6;    // Degrees from horizontal
    public static final double CAMERA_YAW                       = -2.4946892723;
    public static final Transform3d CAMERA_TRANSFORM3D          = new Transform3d(
        new Translation3d(CAMERA_Y_OFFSET*TrcUtil.METERS_PER_INCH, -CAMERA_X_OFFSET*TrcUtil.METERS_PER_INCH,
                          CAMERA_HEIGHT*TrcUtil.METERS_PER_INCH),
        new Rotation3d(0.0, Math.toRadians(-CAMERA_PITCH), Math.toRadians(-CAMERA_YAW)));
    public static final double APRILTAG_SIZE                    = 6.0 / TrcUtil.INCHES_PER_METER;   //  in meters
    // Camera: Logitech C310
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

    // Tuned 3/02/2023
    public static final double SWERVE_INCHES_PER_COUNT          = 9.3802993133e-4;
    public static final double SWERVE_KP                        = 0.016;
    public static final double SWERVE_KI                        = 0.0;
    public static final double SWERVE_KD                        = 0.004;
    public static final double SWERVE_KF                        = 0.0;
    public static final double SWERVE_TOLERANCE                 = 2.0;

    public static final double GYRO_TURN_KP                     = 0.0075;
    public static final double GYRO_TURN_KI                     = 0.0;
    public static final double GYRO_TURN_KD                     = 0.0006;
    public static final double GYRO_TURN_KF                     = 0.0;
    public static final double GYRO_TURN_TOLERANCE              = 2.0;
    public static final double[] GYRO_TILT_THRESHOLDS           = {-15.0, -5.0, 5.0, 15.0};
    public static final double[] DRIVE_DISTANCE_THRESHOLDS      = {20.0};

    public static final double GYRO_PITCH_KP                    = 0.0095;
    public static final double GYRO_PITCH_KI                    = 0.0;
    public static final double GYRO_PITCH_KD                    = 0.001;
    public static final double GYRO_PITCH_KF                    = 0.0;
    public static final double GYRO_PITCH_TOLERANCE             = 2.0;
    public static final double GYRO_PITCH_SETTLING_TIME         = 0.2;
    public static final double GYRO_PITCH_MAX_PID_POWER         = 0.2;
    public static final double GYRO_PITCH_PID_RAMP_RATE         = 0.2;

    public static final double GYRO_ASSIST_TURN_GAIN            = 0.1;

    public static final double X_TIPPING_KP                     = 0.01;
    public static final double X_TIPPING_KI                     = 0.0;
    public static final double X_TIPPING_KD                     = 0.0;
    public static final double X_TIPPING_TOLERANCE              = 10.0;
    public static final double X_TIPPING_SETTLING_TIME          = 0.2;

    public static final double Y_TIPPING_KP                     = 0.01;
    public static final double Y_TIPPING_KI                     = 0.0;
    public static final double Y_TIPPING_KD                     = 0.0;
    public static final double Y_TIPPING_TOLERANCE              = 10.0;
    public static final double Y_TIPPING_SETTLING_TIME          = 0.2;

    public static final double ROBOT_MAX_VELOCITY               = 180.0;
    public static final double ROBOT_MAX_ACCELERATION           = 2100.0;
    public static final double ROBOT_MAX_TURN_RATE              = 1000.0;
    public static final double ROBOT_VEL_KP                     = 0.0;
    public static final double ROBOT_VEL_KI                     = 0.0;
    public static final double ROBOT_VEL_KD                     = 0.0;
    // KF should be set to the reciprocal of max tangential velocity (time to travel unit distance), units: sec./in.
    public static final double ROBOT_VEL_KF                     = 1.0 / ROBOT_MAX_VELOCITY;

    public static final double DRIVE_SLOW_SCALE                 = 0.2;
    public static final double TURN_SLOW_SCALE                  = 0.15;
    public static final double DRIVE_MEDIUM_SCALE               = 0.65;
    public static final double TURN_MEDIUM_SCALE                = 0.3;
    public static final double DRIVE_FAST_SCALE                 = 1.0;
    public static final double TURN_FAST_SCALE                  = 1.0;

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
    public static final double FALCON_MAX_RPM                   = 6380.0;
    public static final double STEER_GEAR_RATIO                 = (24.0/12.0) * (72.0/14.0);
    public static final double STEER_MOTOR_CPR                  = FALCON_CPR * STEER_GEAR_RATIO;
    public static final double STEER_DEGREES_PER_COUNT          = 360.0 / STEER_MOTOR_CPR;
    // ((theoretical max rpm * speed loss constant / gear ratio) / 60 sec/min) * 360 deg/rev
    public static final double STEER_MAX_VEL                    = (FALCON_MAX_RPM*0.81/STEER_GEAR_RATIO/60.0)*360.0;

    public static final double STEER_MAX_REQ_VEL                = 1000.0;   // deg/sec. max commanded velocity, not necessarily max vel
    public static final double STEER_MAX_ACCEL                  = 5000.0;   // deg/sec^2

    // Zeroes are normalized offsets which are in the unit of percentage revolution (0.0 to 1.0).
    // This is a backup if file is not found: LF, RF, LB, RB.
    // 3/05/2023 13:35
    public static final double[] STEER_ZEROS                    = new double[] {0.712430, 0.279410, 0.900320, 0.445040};

    public static final double STEER_MAX_VEL_COUNT_PER_100MS    = (STEER_MAX_VEL / STEER_DEGREES_PER_COUNT) / 10.0;
    // public static final TrcPidController.PidCoefficients magicSteerCoeff =
    //     new TrcPidController.PidCoefficients(2.0, 0.01, 0.0, 1023.0 / STEER_MAX_VEL_COUNT_PER_100MS, 5.0 / STEER_DEGREES_PER_COUNT);
    public static final double STEER_KP                         = 0.3;
    public static final double STEER_KI                         = 0.0;//0.01;
    public static final double STEER_KD                         = 0.0;
    // kF set to Motion Magic recommendation.
    public static final double STEER_KF                         = 0.0;//1023.0 / STEER_MAX_VEL_COUNT_PER_100MS;
    // iZone set to within 5 steering degrees.
    public static final double STEER_IZONE                      = 0.0;//5.0 / STEER_DEGREES_PER_COUNT;
    public static final TrcPidController.PidCoefficients steerCoeffs =
        new TrcPidController.PidCoefficients(STEER_KP, STEER_KI, STEER_KD, STEER_KF, STEER_IZONE);

    public static final double PPD_FOLLOWING_DISTANCE           = 12.0;
    public static final double PPD_POS_TOLERANCE                = 2.0;
    public static final double PPD_TURN_TOLERANCE               = 2.0;
    public static final double PPD_MOVE_DEF_OUTPUT_LIMIT        = 0.5;
    public static final double PPD_ROT_DEF_OUTPUT_LIMIT         = 0.5;

    //
    // Other subsystems.
    //

    // Elevator subsystem.
    public static final int ELEVATOR_ZERO                       = 1635;
    public static final double ELEVATOR_AUTOSTART_OFFSET        = 3.7;
    public static final int NEO_CPR                             = 42;
    public static final double NEO_NOLOAD_RPM                   = 5676.0;
    public static final boolean ELEVATOR_MOTOR_INVERTED         = true;
    public static final boolean ELEVATOR_LOWER_LIMIT_INVERTED   = true;
    public static final boolean ELEVATOR_UPPER_LIMIT_INVERTED   = true;
    public static final double ELEVATOR_CHAIN_PITCH             = 0.25;     // in inches
    public static final double ELEVATOR_SPROCKET_TEETH          = 16.0;
    public static final double ELEVATOR_GEAR_RATIO              = 25.0;
    public static final double ELEVATOR_INCHES_PER_COUNT        =
       (ELEVATOR_CHAIN_PITCH * ELEVATOR_SPROCKET_TEETH) / ELEVATOR_GEAR_RATIO;
    public static final double ELEVATOR_OFFSET                  = 0.0;
    public static final double ELEVATOR_MIN_POS                 = 0.0;
    public static final double ELEVATOR_MAX_POS                 = 29.0;
    public static final double ELEVATOR_KP                      = 0.5;
    public static final double ELEVATOR_KI                      = 0.0;
    public static final double ELEVATOR_KD                      = 0.0;
    public static final double ELEVATOR_KF                      = 0.0;
    public static final double ELEVATOR_IZONE                   = 0.0;
    public static final double ELEVATOR_TOLERANCE               = 1.0;
    public static final double ELEVATOR_CAL_POWER               = -0.4;
    public static final double ELEVATOR_PRESET_TOLERANCE        = 2.0;      // in inches
    public static final double ELEVATOR_SAFE_HEIGHT             = 10.0; //TODO: Determine
    public static final double[] ELEVATOR_TRIGGERS              = {22.0};
    // Important: presets must be sorted in increasing order.
    public static final double[] elevatorPresets                =
    {
        0.0, 6.0, 12.0, 18.0, 24.0, 28.0
    };
    //TODO: tune these values
    public static final double[] elevatorConeScoringPresets     =
    {
        0, 12.5, 28.0
    };
    //TODO: tune these values
    public static final double[] elevatorCubeScoringPresets     =
    {
       0, 0, 22
    };

    // Arm subsystem.
    // public static final double ARM_MAX_VEL                      = (FALCON_MAX_RPM*0.81/STEER_GEAR_RATIO/60.0)*360.0;
    public static final int ARM_ENCODER_CPR                     = 4096;
    public static final int ARM_ZERO                            = 1683;

    public static final boolean ARM_MOTOR_INVERTED              = false;
    public static final boolean ARM_LOWER_LIMIT_INVERTED        = true;
    public static final boolean ARM_UPPER_LIMIT_INVERTED        = true;
    public static final double ARM_DEGS_PER_COUNT               = 360.0 / ARM_ENCODER_CPR;
    public static final double ARM_OFFSET                       = -37.0;    // in degrees
    public static final double ARM_LOW_POS                      = 8.0;
    public static final double ARM_MIN_POS                      = ARM_LOW_POS;
    public static final double ARM_MAX_POS                      = 90.0;
    public static final double ARM_SAFE_RANGE                   = ARM_MAX_POS - ARM_LOW_POS;
    public static final double ARM_KP                           = 0.02;
    public static final double ARM_KI                           = 0.0;
    public static final double ARM_KD                           = 0.0;
    public static final double ARM_KF                           = 0.0005;
    public static final double ARM_IZONE                        = 10.0;
    public static final double ARM_TOLERANCE                    = 1.0;
    public static final double ARM_CAL_POWER                    = -0.2;
    public static final double ARM_MAX_POWER                    = 0.5;
    public static final double ARM_MAX_GRAVITY_COMP_POWER       = 0.0;
    public static final double ARM_PRESET_TOLERANCE             = 5.0;
    public static final double ARM_PICKUP_POSITION              = 40.0;     // 40 deg TODO: Verify
    public static final double ARM_TRAVEL_POSITION              = 30.0;     //TODO
    public static final double ARM_SAFE_POSITION                = 10.0;     //TODO
    // Important: presets must be sorted in increasing order.
    public static final double[] armPresets                     =
    {
        ARM_LOW_POS, ARM_TRAVEL_POSITION, ARM_PICKUP_POSITION, ARM_MAX_POS
    };

    //TODO: tune these values, random numbers
    public static final double[] armConeScorePresets            =
    {
        20, ARM_MAX_POS, ARM_MAX_POS
    };
    //TODO: tune these values, random numbers
    public static final double[] armCubeScorePresets            =
    {
        20, ARM_MAX_POS, ARM_MAX_POS
    };

    // Intake subsystem.
    public static final double INTAKE_CUBE_PICKUP_POWER         = 0.8;
    public static final double INTAKE_CONE_PICKUP_POWER         = 0.9;
    public static final double INTAKE_SPIT_POWER                = -0.5;

    // Grabber subsystem.
    public static final double GRABBER_GRAB_DURATION            = 0.2;
    public static final double GRABBER_RELEASE_DURATION         = 0.2;

}   //class RobotParams
