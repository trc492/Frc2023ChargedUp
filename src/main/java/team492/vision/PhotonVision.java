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

package team492.vision;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcTimer;
import TrcFrcLib.frclib.FrcPhotonVision;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import team492.RobotParams;
import team492.subsystems.LEDIndicator;

/**
 * This class is a thin wrapper extending FrcPhotonVision that provides additional game specific functionalities.
 */
public class PhotonVision extends FrcPhotonVision
{
    private static final String moduleName = "PhotonVision";
    private static final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
    private static final boolean debugEnabled = false;

    public enum PipelineType
    {
        APRILTAG(0),
        CONE(1),
        CUBE(2),
        POLE(3);

        public int pipelineIndex;

        PipelineType(int value)
        {
            pipelineIndex = value;
        }

        public static PipelineType getType(int index)
        {
            PipelineType type = null;

            for (PipelineType pipelineType: PipelineType.values())
            {
                if (index == pipelineType.pipelineIndex)
                {
                    type = pipelineType;
                }
            }

            return type;
        }   //getType
    }   //enum PipelineType

    private final LEDIndicator ledIndicator;
    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final PhotonPoseEstimator poseEstimator;
    private PipelineType currPipeline;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param cameraName specifies the photon vision camera name.
     * @param ledIndicator specifies the LEDIndicator object, can be null if none provided.
     * @param tracer specifies the tracer for trace info, null if none provided.
     */
    public PhotonVision(String cameraName, LEDIndicator ledIndicator, TrcDbgTrace tracer)
    {
        super(cameraName, RobotParams.CAMERA_HEIGHT, RobotParams.CAMERA_PITCH, tracer);
        this.ledIndicator = ledIndicator;

        double startTime = TrcTimer.getModeElapsedTime();
        try
        {
            aprilTagFieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            poseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, this, RobotParams.CAMERA_TRANSFORM3D);
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }
        catch (IOException e)
        {
            throw new RuntimeException("Failed to load AprilTag field layout info.");
        }
        double endTime = TrcTimer.getModeElapsedTime();

        if (debugEnabled)
        {
            globalTracer.traceInfo(
                moduleName, "[%.3f] Loading AprilTag field layout took %.3f sec.", endTime, endTime - startTime);
        }

        setPipeline(PipelineType.APRILTAG);
    }   //PhotonVision

    /**
     * This method returns the best detected object and set the LED to indicate type detected object type.
     *
     * @return best detected object.
     */
    @Override
    public DetectedObject getBestDetectedObject()
    {
        DetectedObject detectedObject = super.getBestDetectedObject();

        if (detectedObject != null && ledIndicator != null)
        {
            // ledIndicator.setVisionDetectedObject(getPipeline());
        }

        return detectedObject;
    }   //getBestDetectedObject

    /**
     * This method returns the 3D field location of the AprilTag with its given ID.
     *
     * @param aprilTagId sepcifies the AprilTag ID to retrieve its field location.
     * @return 3D location of the AprilTag.
     */
    public Pose3d getAprilTagPose(int aprilTagId)
    {
        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(aprilTagId);
        return tagPose.isPresent()? tagPose.get(): null;
    }   //getAprilTagPose

    /**
     * This method returns the absolute field location of the camera with the given detected AprilTag object.
     *
     * @param detectedObj specifies the AprilTag object detected by the camera.
     * @return camera's absolute field location.
     */
    public TrcPose2D getRobotFieldPosition(DetectedObject detectedObj)
    {
        final String funcName = "getRobotFieldPosition";
        TrcPose2D robotPose = null;
        int aprilTagId = detectedObj.target.getFiducialId();
        // aprilTagPose is the absolute field position of the AprilTag.
        Pose3d aprilTagPose = getAprilTagPose(aprilTagId);

        if (aprilTagPose != null)
        {
            // camPose3d is the absolute field position of the camera.
            Pose3d camPose3d = aprilTagPose.transformBy(detectedObj.target.getBestCameraToTarget().inverse());
            // robotPose3d is the absolute 3D field position of the robot centroid on the ground.
            Pose3d robotPose3d = camPose3d.transformBy(RobotParams.CAMERA_TRANSFORM3D.inverse());
            // robotPose is the absolute field position of the robot adjusted to the robot coordinate system.
            robotPose = DetectedObject.pose3dToTrcPose2D(robotPose3d);

            if (debugEnabled)
            {
                globalTracer.traceInfo(
                    funcName, "[%d] camPose3d=%s, robotPose3d=%s, RobotPose=%s",
                    aprilTagId, camPose3d, robotPose3d, robotPose);
            }
        }

        return robotPose;
    }   //getRobotFieldPosition

    /**
     * This method uses the PhotonVision Pose Estimator to get an estimated absolute field position of the robot.
     *
     * @return absolute robot field position, can be null if not provided.
     */
    public TrcPose2D getEstimatedFieldPosition(TrcPose2D robotPose)
    {
        TrcPose2D estimatedRobotPose = null;

        if (poseEstimator != null)
        {
            TrcDbgTrace.globalTraceInfo("getEstPose", "has poseEstimator");
            if (robotPose != null)
            {
                poseEstimator.setReferencePose(DetectedObject.trcPose2DToPose3d(robotPose));
            }
            Optional<EstimatedRobotPose> optionalPose = poseEstimator.update();
            TrcDbgTrace.globalTraceInfo("getEstPose", "optionalPose=%s", optionalPose.isPresent());
            if (optionalPose.isPresent())
            {
                estimatedRobotPose = DetectedObject.pose3dToTrcPose2D(optionalPose.get().estimatedPose);
                TrcDbgTrace.globalTraceInfo("getEstPose", "estPose=%s", estimatedRobotPose);
            }
        }

        return estimatedRobotPose;
    }   //getEstimatedFieldPosition

    /**
     * This method sets the active pipeline type used in the LimeLight.
     *
     * @param pipelineType specifies the pipeline to activate in the LimeLight.
     */
    public void setPipeline(PipelineType pipelineType)
    {
        if (pipelineType != currPipeline)
        {
            currPipeline = pipelineType;
            setPipelineIndex(pipelineType.pipelineIndex);
            setLED(pipelineType == PipelineType.POLE? VisionLEDMode.kOn: VisionLEDMode.kOff);
        }
    }   //setPipeline

    /**
     * This method returns the active pipeline of the LimeLight.
     *
     * @return active pipeline.
     */
    public PipelineType getPipeline()
    {
        currPipeline = PipelineType.getType(getPipelineIndex());
        return currPipeline;
    }   //getPipeline

    //
    // Implements FrcPhotonVision abstract methods.
    //

    /**
     * This method returns the ground offset of the detected target.
     *
     * @return target ground offset.
     */
    public double getTargetHeight(PhotonTrackedTarget target)
    {
        double targetHeight = 0.0;
        PipelineType pipelineType = getPipeline();

        switch (pipelineType)
        {
            case APRILTAG:
                if (target != null)
                {
                    targetHeight = getAprilTagPose(target.getFiducialId()).getZ();
                }
                break;

            case CUBE:
                targetHeight = RobotParams.CUBE_HALF_HEIGHT;
                break;

            case CONE:
                targetHeight = RobotParams.CONE_HALF_HEIGHT;
                break;

            case POLE:
                targetHeight = RobotParams.LOW_POLE_TAPE_HEIGHT;
                break;
        }

        return targetHeight;
    }   //getTargetHeight

}   //class PhotonVision
