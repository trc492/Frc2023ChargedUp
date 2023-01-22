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

import java.io.IOException;
import java.util.Optional;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcUtil;
import TrcFrcLib.frclib.FrcPhotonVision;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * This class is a thin wrapper extending FrcPhotonVision that provides additional game specific functionalities.
 */
public class PhotonVision extends FrcPhotonVision
{
    private static final String moduleName = "PhotonVision";
    private static final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
    private static final  boolean debugEnabled = true;

    private final AprilTagFieldLayout aprilTagFieldLayout;
    // private final AprilTagPoseEstimator poseEstimator;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param cameraName specifies the photon vision camera name.
     * @param tracer specifies the tracer for trace info, null if none provided.
     */
    public PhotonVision(String cameraName, TrcDbgTrace tracer)
    {
        super(cameraName, tracer);

        double startTime = TrcTimer.getModeElapsedTime();
        try
        {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        }
        catch (IOException e)
        {
            throw new RuntimeException("Failed to read AprilTag field layout info.");
        }
        double endTime = TrcTimer.getModeElapsedTime();
        if (debugEnabled)
        {
            globalTracer.traceInfo(
                moduleName, "[%.3f] Loading AprilTag field layout took %.3f sec.", endTime, endTime - startTime);
        }

        // poseEstimator = new AprilTagPoseEstimator(
        //     new AprilTagPoseEstimator.Config(
        //         RobotParams.APRILTAG_SIZE, RobotParams.APRILTAG_FX, RobotParams.APRILTAG_FY,
        //         RobotParams.APRILTAG_CX, RobotParams.APRILTAG_CY));
    }   //FrcPhotonVision

    /**
     * This method returns the 3D field location of the AprilTag with its given ID.
     *
     * @param aprilTagId sepcifies the AprilTag ID to retrieve its field location.
     * @return 3D location of the 
     */
    private Pose3d getAprilTagPose(int aprilTagId)
    {
        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(aprilTagId);
        return tagPose.isPresent()? tagPose.get(): null;
    }   //getAprilTagPose

    /**
     * This method returns the absolute field location of the camera with the given detected AprilTag object.
     *
     * @param aprilTagObj specifies the AprilTag object detected by the camera.
     * @return camera's absolute field location.
     */
    public TrcPose2D getRobotFieldPosition(DetectedObject aprilTagObj)
    {
        final String funcName = "getRobotFieldPosition";
        TrcPose2D robotPose = null;

        int aprilTagId = aprilTagObj.target.getFiducialId();
        Pose3d aprilTagPose = getAprilTagPose(aprilTagId);

        if (aprilTagPose != null)
        {
            Pose3d camPose3d = aprilTagPose.transformBy(aprilTagObj.targetTransform.inverse());
            Pose2d robotPose2d = camPose3d.transformBy(RobotParams.CAMERA_TRANSFORM3D.inverse()).toPose2d();
            robotPose = new TrcPose2D(
                -robotPose2d.getY() * TrcUtil.INCHES_PER_METER,
                robotPose2d.getX() * TrcUtil.INCHES_PER_METER,
                -robotPose2d.getRotation().getDegrees());

            if (debugEnabled)
            {
                globalTracer.traceInfo(
                    funcName, "[%d] AprilTagPose=%s, RobotPose=%s, yaw=%.1f",
                    aprilTagId, aprilTagPose, robotPose, aprilTagObj.target.getYaw());
            }
        }

        return robotPose;
        // poseEstimator.update(gyro.getRotation2d(), leftDist, rightDist);

        // var res = cam.getLatestResult();
        // if (res.hasTargets()) {
        //     var imageCaptureTime = res.getTimestampSeconds();
        //     var camToTargetTrans = res.getBestTarget().getBestCameraToTarget();
        //     var camPose = Constants.kFarTargetPose.transformBy(camToTargetTrans.inverse());
        //     m_poseEstimator.addVisionMeasurement(
        //             camPose.transformBy(Constants.kCameraToRobot).toPose2d(), imageCaptureTime);
        // }        
    }   //getRobotFieldPosition

}   //class PhotonVision