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

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcFrcLib.frclib.FrcLimeLightVision;
import TrcFrcLib.frclib.FrcRemoteVisionProcessor;
import team492.RobotParams;

public class LimeLightVision extends FrcLimeLightVision
{
    public LimeLightVision(String instanceName, TrcDbgTrace tracer)
    {
        super(instanceName, tracer);
        selectPipeline(0);
        setDepthApproximator(
            "ty",
            y -> (RobotParams.VISION_TARGET_HEIGHT - RobotParams.CAMERA_HEIGHT) /
                 Math.tan(Math.toRadians(y + RobotParams.CAMERA_PITCH)));
        setOffsets(RobotParams.CAMERA_X_OFFSET, RobotParams.CAMERA_Y_OFFSET);
        setFreshnessTimeout(RobotParams.CAMERA_DATA_TIMEOUT);
        setRingLightEnabled(RingLightMode.OFF);
    }

    public boolean targetAcquired()
    {
        return targetDetected();
    }   //targetAcquired

    public double getTargetHorizontalAngle()
    {
        return getHeading();
    }   //getTargetHorizontalAngle

    public double getTargetVerticalAngle()
    {
        return getElevation() + RobotParams.CAMERA_PITCH;
    }   //getTargetVerticalAngle

    public double getTargetDistance()
    {
        return getTargetDepth();
    }   //getTargetDistance

    public FrcRemoteVisionProcessor.RelativePose getLastPose()
    {
        return getLastPose();
    }

    public void setEnabled(boolean enabled)
    {
        setEnabled(enabled);
        setRingLightEnabled(enabled);
    }

    public FrcRemoteVisionProcessor.RelativePose getMedianPose(int numFrames, boolean requireAll)
    {
        return getMedianPose(numFrames, requireAll);
    }

}   //class LimeLightVision
