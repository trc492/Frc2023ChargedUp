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

import TrcFrcLib.frclib.FrcLimeLightVisionProcessor;
import TrcFrcLib.frclib.FrcRemoteVisionProcessor;
import TrcFrcLib.frclib.FrcLimeLightVisionProcessor.RingLightMode;

public class VisionTargeting
{
    public final FrcLimeLightVisionProcessor vision;

    public VisionTargeting()
    {
        vision = new FrcLimeLightVisionProcessor("LimeLight");
        vision.selectPipeline(0);
        vision.setDepthApproximator(
            "ty",
            y -> (RobotParams.VISION_HIGH_TARGET_HEIGHT - RobotParams.CAMERA_HEIGHT) /
                 Math.tan(Math.toRadians(y + RobotParams.CAMERA_ANGLE)));
        vision.setOffsets(RobotParams.CAMERA_X_OFFSET, RobotParams.CAMERA_Y_OFFSET);
        vision.setFreshnessTimeout(RobotParams.CAMERA_DATA_TIMEOUT);
        vision.setRingLightEnabled(RingLightMode.OFF);
    }

    public void setLightEnabled(boolean enabled)
    {
        vision.setRingLightEnabled(enabled? RingLightMode.ON: RingLightMode.OFF);
    }

    public boolean targetAcquired()
    {
        return vision.targetDetected();
    }   //targetAcquired

    public double getTargetHorizontalAngle()
    {
        return vision.getHeading();
    }   //getTargetHorizontalAngle

    public double getTargetVerticalAngle()
    {
        return vision.getElevation() + RobotParams.CAMERA_ANGLE;
    }   //getTargetVerticalAngle

    public double getTargetDistance()
    {
        return vision.getTargetDepth() * RobotParams.VISION_DISTANCE_FUDGE_FACTOR;
    }   //getTargetDistance

    public FrcRemoteVisionProcessor.RelativePose getLastPose()
    {
        return vision.getLastPose();
    }

    public void setEnabled(boolean enabled)
    {
        vision.setEnabled(enabled);
        setLightEnabled(enabled);
    }

    public FrcRemoteVisionProcessor.RelativePose getMedianPose(int numFrames, boolean requireAll)
    {
        return vision.getMedianPose(numFrames, requireAll);
    }

}   //class VisionTargeting
