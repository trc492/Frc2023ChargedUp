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
import TrcFrcLib.frclib.FrcPhotonVision;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;

public class PhotonVision extends FrcPhotonVision
{
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
        try
        {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        }
        catch (IOException e)
        {
            throw new RuntimeException("Failed to read AprilTag field layout info.");
        }

        // poseEstimator = new AprilTagPoseEstimator(
        //     new AprilTagPoseEstimator.Config(
        //         RobotParams.APRILTAG_SIZE, RobotParams.APRILTAG_FX, RobotParams.APRILTAG_FY,
        //         RobotParams.APRILTAG_CX, RobotParams.APRILTAG_CY));
    }   //FrcPhotonVision

    private Pose3d getAprilTagPose(int aprilTagId)
    {
        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(aprilTagId);
        return tagPose.isPresent()? tagPose.get(): null;
    }   //getAprilTagPose

    public Pose3d getRobotFieldPosition(DetectedObject aprilTagObj)
    {
        Pose3d robotPose = null;
        Pose3d aprilTagPose = getAprilTagPose(aprilTagObj.target.getFiducialId());

        if (aprilTagPose != null)
        {
            aprilTagPose.transformBy(aprilTagObj.targetTransform.inverse());
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
