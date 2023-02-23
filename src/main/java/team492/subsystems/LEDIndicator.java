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
import team492.drivebases.RobotDrive;

public class LEDIndicator
{
    private static final TrcAddressableLED.Pattern nominalPattern = // Black
        new TrcAddressableLED.Pattern("Nominal", new FrcColor(0, 0, 0), RobotParams.NUM_LEFT_LEDS);

    private static final TrcAddressableLED.Pattern fieldOrientedPattern = // Cyan
        new TrcAddressableLED.Pattern("FieldOriented", new FrcColor(0, 63, 63), RobotParams.NUM_LEFT_LEDS);
    private static final TrcAddressableLED.Pattern robotOrientedPattern = // Red
        new TrcAddressableLED.Pattern("RobotOriented", new FrcColor(63, 0, 0), RobotParams.NUM_LEFT_LEDS);
    private static final TrcAddressableLED.Pattern inverseOrientedPattern = // Magenta
        new TrcAddressableLED.Pattern("InverseOriented", new FrcColor(63, 0, 63), RobotParams.NUM_LEFT_LEDS);

    private static final TrcAddressableLED.Pattern[] leftPriorities =
        new TrcAddressableLED.Pattern[]
        {
            nominalPattern,
            inverseOrientedPattern,
            robotOrientedPattern,
            fieldOrientedPattern,
        };
    private static final TrcAddressableLED.Pattern[] rightPriorities =
        new TrcAddressableLED.Pattern[]
        {
        };

    private final FrcAddressableLED leftLed;
    private final FrcAddressableLED rightLed;

    /**
     * Constructor: Create an instance of the object.
     */
    public LEDIndicator()
    {
        leftLed = new FrcAddressableLED("LeftLED", RobotParams.NUM_LEFT_LEDS, RobotParams.PWM_CHANNEL_LEFT_LED);
        rightLed = new FrcAddressableLED("RightLED", RobotParams.NUM_RIGHT_LEDS, RobotParams.PWM_CHANNEL_RIGHT_LED);
        reset();
    }   //LEDIndicator

    /**
     * This method resets the LED strip to the nominal pattern.
     */
    public void reset()
    {
        leftLed.setEnabled(true);
        leftLed.setPatternPriorities(leftPriorities);
        leftLed.reset();
        leftLed.resetAllPatternStates();
        leftLed.setPatternState(nominalPattern, true);
        rightLed.setEnabled(true);
        rightLed.setPatternPriorities(rightPriorities);
        rightLed.reset();
        rightLed.resetAllPatternStates();
        rightLed.setPatternState(nominalPattern, true);
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
                leftLed.setPatternState(inverseOrientedPattern, true);
                leftLed.setPatternState(robotOrientedPattern, false);
                leftLed.setPatternState(fieldOrientedPattern, false);
                break;

            case ROBOT:
                leftLed.setPatternState(inverseOrientedPattern, false);
                leftLed.setPatternState(robotOrientedPattern, true);
                leftLed.setPatternState(fieldOrientedPattern, false);
                break;

            case FIELD:
                leftLed.setPatternState(inverseOrientedPattern, false);
                leftLed.setPatternState(robotOrientedPattern, false);
                leftLed.setPatternState(fieldOrientedPattern, true);
                break;
        }
    }   //setDriveOrientation

}   //class LEDIndicator
