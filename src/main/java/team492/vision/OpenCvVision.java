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

import java.util.Comparator;

import org.opencv.imgproc.Imgproc;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcHomographyMapper;
import TrcCommonLib.trclib.TrcOpenCvColorBlobPipeline;
import TrcCommonLib.trclib.TrcOpenCvDetector;
import TrcCommonLib.trclib.TrcOpenCvPipeline;
import TrcCommonLib.trclib.TrcVisionTargetInfo;
import TrcFrcLib.frclib.FrcOpenCvAprilTagPipeline;
import TrcFrcLib.frclib.FrcOpenCvDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import team492.RobotParams;

public class OpenCvVision extends FrcOpenCvDetector
{
    private static final int colorConversion = Imgproc.COLOR_RGBA2RGB;
    private static final double[] colorThresholdsPole = {100.0, 255.0, 0.0, 100.0, 0.0, 60.0};
    private static final double[] colorThresholdsCone = {100.0, 255.0, 0.0, 100.0, 0.0, 60.0};
    private static final double[] colorThresholdsCube = {0.0, 60.0, 0.0, 100.0, 100, 255.0};

    public enum ObjectType
    {
        APRILTAG, POLE, CONE, CUBE, NONE;

        static ObjectType nextObjectType(ObjectType objType)
        {
            ObjectType nextObjType;

            switch (objType)
            {
                case APRILTAG:
                    nextObjType = POLE;
                    break;

                case POLE:
                    nextObjType = CONE;
                    break;

                case CONE:
                    nextObjType = CUBE;
                    break;

                case CUBE:
                    nextObjType = NONE;
                    break;

                default:
                case NONE:
                    nextObjType = APRILTAG;
                    break;
            }

            return nextObjType;
        }   //nextObjectType

    }   //enum ObjectType

    private final TrcDbgTrace tracer;
    private final TrcOpenCvPipeline<DetectedObject<?>> aprilTagPipeline;
    private final TrcOpenCvPipeline<DetectedObject<?>> polePipeline;
    private final TrcOpenCvPipeline<DetectedObject<?>> conePipeline;
    private final TrcOpenCvPipeline<DetectedObject<?>> cubePipeline;
    private ObjectType objectType = ObjectType.NONE;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param numImageBuffers specifies the number of image buffers to allocate.
     * @param imageWidth specifies the width of the camera image.
     * @param imageHeight specifies the height of the camera image.
     * @param cameraRect specifies the camera rectangle for Homography Mapper, can be null if not provided.
     * @param worldRect specifies the world rectangle for Homography Mapper, can be null if not provided.
     * @param cvSink specifies the object to capture the video frames.
     * @param cvSource specifies the object to stream video output.
     * @param tracer specifies the tracer for trace info, null if none provided.
     */
    public OpenCvVision(
        String instanceName, int numImageBuffers, int imageWidth, int imageHeight,
        TrcHomographyMapper.Rectangle cameraRect, TrcHomographyMapper.Rectangle worldRect,
        CvSink cvSink, CvSource cvSource, TrcDbgTrace tracer)
    {
        super(instanceName, numImageBuffers, imageWidth, imageHeight,  cameraRect, worldRect, cvSink, cvSource,
              tracer);

        this.tracer = tracer;
        TrcOpenCvColorBlobPipeline.FilterContourParams poleFilterContourParams =
            new TrcOpenCvColorBlobPipeline.FilterContourParams()
                .setMinArea(5000.0)
                .setMinPerimeter(500.0)
                .setWidthRange(100.0, 1000.0)
                .setHeightRange(250.0, 10000.0)
                .setSolidityRange(0.0, 100.0)
                .setVerticesRange(0.0, 1000.0)
                .setAspectRatioRange(0.0, 1000.0);
        TrcOpenCvColorBlobPipeline.FilterContourParams coneFilterContourParams =
            new TrcOpenCvColorBlobPipeline.FilterContourParams()
                .setMinArea(10000.0)
                .setMinPerimeter(200.0)
                .setWidthRange(100.0, 1000.0)
                .setHeightRange(100.0, 1000.0)
                .setSolidityRange(0.0, 100.0)
                .setVerticesRange(0.0, 1000.0)
                .setAspectRatioRange(0.0, 1000.0);
        TrcOpenCvColorBlobPipeline.FilterContourParams cubeFilterContourParams =
            new TrcOpenCvColorBlobPipeline.FilterContourParams()
                .setMinArea(10000.0)
                .setMinPerimeter(200.0)
                .setWidthRange(100.0, 1000.0)
                .setHeightRange(100.0, 1000.0)
                .setSolidityRange(0.0, 100.0)
                .setVerticesRange(0.0, 1000.0)
                .setAspectRatioRange(0.0, 1000.0);

        aprilTagPipeline = new FrcOpenCvAprilTagPipeline(
            "tag16h5", null, new AprilTagPoseEstimator.Config(
                RobotParams.APRILTAG_SIZE, RobotParams.WEBCAM_FX, RobotParams.WEBCAM_FY, RobotParams.WEBCAM_CX,
                RobotParams.WEBCAM_CY),
            tracer);
        polePipeline = new TrcOpenCvColorBlobPipeline(
            "polePipeline", colorConversion, colorThresholdsPole, poleFilterContourParams, tracer);
        conePipeline = new TrcOpenCvColorBlobPipeline(
            "conePipeline", colorConversion, colorThresholdsCone, coneFilterContourParams, tracer);
        cubePipeline = new TrcOpenCvColorBlobPipeline(
            "cubePipeline", colorConversion, colorThresholdsCube, cubeFilterContourParams, tracer);
    }   //OpenCvVision

    /**
     * This method updates the pipeline to detect the currently selected object type.
     */
    private void updatePipeline()
    {
        if (tracer != null)
        {
            tracer.traceInfo("updatePipeline", "objType=%s", objectType);
        }

        switch (objectType)
        {
            case APRILTAG:
                setPipeline(aprilTagPipeline);
                break;

            case POLE:
                setPipeline(polePipeline);
                break;

            case CONE:
                setPipeline(conePipeline);
                break;

            case CUBE:
                setPipeline(cubePipeline);
                break;

            case NONE:
                setPipeline(null);
                break;
        }
    }   //updatePipeline

    /**
     * This method sets the object type to detect.
     *
     * @param objType specifies the object type to detect.
     */
    public void setDetectObjectType(ObjectType objType)
    {
        objectType = objType;
        updatePipeline();
    }   //setDetectObjectType

    /**
     * This method sets the detect object type to the next type.
     */
    public void setNextObjectType()
    {
        setDetectObjectType(ObjectType.nextObjectType(objectType));
    }   //setNextObjectType

    /**
     * This method returns the selected detect object type.
     *
     * @return selected detect object type.
     */
    public ObjectType getDetectObjectType()
    {
        return objectType;
    }   //getDetectObjectType

    /**
     * This method sets the intermediate mat of the pipeline as the video output mat and optionally annotate the
     * detected rectangle on it.
     *
     * @param intermediateStep specifies the intermediate mat used as video output (1 is the original mat, 0 to
     *        disable video output if supported).
     * @param annotate specifies true to annotate detected rectangles on the output mat, false otherwise.
     *        This parameter is ignored if intermediateStep is 0.
     */
    public void setVideoOutput(int intermediateStep, boolean annotate)
    {
        getPipeline().setVideoOutput(intermediateStep, annotate);
    }   //setVideoOutput

    /**
     * This method returns an array of detected targets from Grip vision.
     *
     * @param filter specifies the filter to call to filter out false positive targets.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @return array of detected target info.
     */
    public TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> getTargetInfo(
        FilterTarget filter, Comparator<? super TrcVisionTargetInfo<DetectedObject<?>>> comparator)
    {
        TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>>[] targets =
            getDetectedTargetsInfo(filter, comparator, RobotParams.VISION_TARGET_HEIGHT, RobotParams.CAMERA_HEIGHT);

        return targets != null? targets[0]: null;
    }   //getTargetInfo

}   //class OpenCvVision
