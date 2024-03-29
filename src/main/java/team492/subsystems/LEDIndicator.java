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

package team492.subsystems;

import TrcCommonLib.trclib.TrcAddressableLED;
import TrcFrcLib.frclib.FrcAddressableLED;
import TrcFrcLib.frclib.FrcColor;
import team492.RobotParams;
import team492.FrcAuto.ObjectType;
import team492.drivebases.RobotDrive;
import team492.vision.PhotonVision.PipelineType;

public class LEDIndicator
{
    private static final TrcAddressableLED.Pattern nominalPattern =             // Black
        new TrcAddressableLED.Pattern("Nominal", new FrcColor(0, 0, 0), RobotParams.NUM_LEDS);

    private static final TrcAddressableLED.Pattern inverseOrientedPattern =     // Blue
        new TrcAddressableLED.Pattern("InverseOriented", new FrcColor(0, 0, 63), RobotParams.NUM_LEDS);
    private static final TrcAddressableLED.Pattern robotOrientedPattern =       // White
        new TrcAddressableLED.Pattern("RobotOriented", new FrcColor(63, 63, 63), RobotParams.NUM_LEDS);
    private static final TrcAddressableLED.Pattern fieldOrientedPattern =       // Cyan
        new TrcAddressableLED.Pattern("FieldOriented", new FrcColor(0, 63, 63), RobotParams.NUM_LEDS);

    private static final TrcAddressableLED.Pattern hasObjectPattern =           // Red
        new TrcAddressableLED.Pattern("hasObject", new FrcColor(63, 0, 0), RobotParams.NUM_LEDS);

    private static final TrcAddressableLED.Pattern scoreCubeLevel2Pattern =     // Bright Magenta
        new TrcAddressableLED.Pattern("scoreLevel2", new FrcColor(255, 0, 255), 45);
    private static final TrcAddressableLED.Pattern scoreCubeLevel1Pattern =     // Bright Magenta
        new TrcAddressableLED.Pattern("scoreLevel1", new FrcColor(255, 0, 255), 30);
    private static final TrcAddressableLED.Pattern scoreCubeLevel0Pattern =     // Bright Magenta
        new TrcAddressableLED.Pattern("scoreLevel0", new FrcColor(255, 0, 255), 15);

    private static final TrcAddressableLED.Pattern scoreConeLevel2Pattern =     // Bright Yellow
        new TrcAddressableLED.Pattern("scoreLevel2", new FrcColor(255, 255, 0), 45);
    private static final TrcAddressableLED.Pattern scoreConeLevel1Pattern =     // Bright Yellow
        new TrcAddressableLED.Pattern("scoreLevel1", new FrcColor(255, 255, 0), 30);
    private static final TrcAddressableLED.Pattern scoreConeLevel0Pattern =     // Bright Yellow
        new TrcAddressableLED.Pattern("scoreLevel0", new FrcColor(255, 255, 0), 15);

    private static final TrcAddressableLED.Pattern detectedConePattern =        // Yellow
        new TrcAddressableLED.Pattern("detectedCone", new FrcColor(63, 63, 0), RobotParams.NUM_LEDS);
    private static final TrcAddressableLED.Pattern detectedCubePattern =        // Magenta
        new TrcAddressableLED.Pattern("detectedCube", new FrcColor(63, 0, 63), RobotParams.NUM_LEDS);
    private static final TrcAddressableLED.Pattern detectedAprilTagPattern =    // Green
        new TrcAddressableLED.Pattern("detectedAprilTag", new FrcColor(0, 63, 0), RobotParams.NUM_LEDS);

    private static final TrcAddressableLED.Pattern[] coneScoreLevelPatterns =
    {
        scoreConeLevel0Pattern, scoreConeLevel1Pattern, scoreConeLevel2Pattern
    };

    private static final TrcAddressableLED.Pattern[] cubeScoreLevelPatterns =
    {
        scoreCubeLevel0Pattern, scoreCubeLevel1Pattern, scoreCubeLevel2Pattern
    };

    private static final TrcAddressableLED.Pattern[] priorities =
        new TrcAddressableLED.Pattern[]
        {
            // Highest priority.
            detectedAprilTagPattern,
            detectedCubePattern,
            detectedConePattern,
            hasObjectPattern,
            scoreConeLevel2Pattern,
            scoreCubeLevel2Pattern,
            scoreConeLevel1Pattern,
            scoreCubeLevel1Pattern,
            scoreConeLevel0Pattern,
            scoreCubeLevel0Pattern,
            fieldOrientedPattern,
            robotOrientedPattern,
            inverseOrientedPattern,
            nominalPattern
            // Lowest priority.
        };

    private final FrcAddressableLED led;

    /**
     * Constructor: Create an instance of the object.
     */
    public LEDIndicator()
    {
        led = new FrcAddressableLED("LED", RobotParams.NUM_LEDS, RobotParams.PWM_CHANNEL_LED);
        reset();
    }   //LEDIndicator

    /**
     * This method resets the LED strip to the nominal pattern.
     */
    public void reset()
    {
        led.setEnabled(true);
        led.setPatternPriorities(priorities);
        led.reset();
        led.resetAllPatternStates();
        led.setPatternState(nominalPattern, true);
    }   //reset

    /**
     * This method sets the LED to indicate the drive orientation mode of the robot.
     *
     * @param orientation specifies the drive orientation mode.
     */
    public void setDriveOrientation(RobotDrive.DriveOrientation orientation)
    {
        switch (orientation)
        {
            case INVERTED:
                led.setPatternState(inverseOrientedPattern, true);
                led.setPatternState(robotOrientedPattern, false);
                led.setPatternState(fieldOrientedPattern, false);
                break;

            case ROBOT:
                led.setPatternState(inverseOrientedPattern, false);
                led.setPatternState(robotOrientedPattern, true);
                led.setPatternState(fieldOrientedPattern, false);
                break;

            case FIELD:
                led.setPatternState(inverseOrientedPattern, false);
                led.setPatternState(robotOrientedPattern, false);
                led.setPatternState(fieldOrientedPattern, true);
                break;
        }
    }   //setDriveOrientation

    /**
     * This method sets the LED to indicate the vision detected object type.
     *
     * @param pipelineType specifies the current vision pipeline type.
     */
    public void setVisionDetectedObject(PipelineType pipelineType)
    {
        switch (pipelineType)
        {
            case APRILTAG:
                led.setPatternState(detectedAprilTagPattern, true, 0.5);
                break;

            case CUBE:
                led.setPatternState(detectedCubePattern, true, 0.5);
                break;

            case CONE:
                led.setPatternState(detectedConePattern, true, 0.5);
                break;

            default:
                break;
        }
    }   //setVisionDetectedObject

    /**
     * This method sets the LED to indicate intake or grabber has an object.
     *
     * @param hasObject specifies true to indicate intake or grabber has an object, false otherwise.
     */
    public void setHasObject(boolean hasObject)
    {
        led.setPatternState(hasObjectPattern, hasObject, 0.5);
    }   //setHasObject

    private void clearScoreLevels(TrcAddressableLED.Pattern[] scoreLevelPatterns)
    {
        // Clear all of them first.
        for (TrcAddressableLED.Pattern pattern: scoreLevelPatterns)
        {
            led.setPatternState(pattern, false);
        }
    }   //clearScoreLevels

    public void setScoreLevel(ObjectType objType, int scoreLevel)
    {
        clearScoreLevels(coneScoreLevelPatterns);
        clearScoreLevels(cubeScoreLevelPatterns);
        led.setPatternState(
            objType == ObjectType.CONE? coneScoreLevelPatterns[scoreLevel]: cubeScoreLevelPatterns[scoreLevel], true);
    }   //setScoreLevel

    public void setIntakeRunning(boolean running, ObjectType objType, int scoreLevel)
    {
        clearScoreLevels(coneScoreLevelPatterns);
        clearScoreLevels(cubeScoreLevelPatterns);
        led.setPatternState(
            objType == ObjectType.CONE? coneScoreLevelPatterns[scoreLevel]: cubeScoreLevelPatterns[scoreLevel], true,
            0.1, running? 0.1: 0.0);
    }

}   //class LEDIndicator
