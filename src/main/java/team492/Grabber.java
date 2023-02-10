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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
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

import TrcCommonLib.trclib.TrcExclusiveSubsystem;
import TrcFrcLib.frclib.FrcPWMTalonSRX;
import TrcFrcLib.frclib.FrcPneumatic;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Grabber implements TrcExclusiveSubsystem
{
    private final FrcPneumatic leftGrabber;
    private final FrcPneumatic rightGrabber;
    private final FrcPWMTalonSRX vacuum;
    
    /**
     * Constructor: Create an instance of the object.
     *
     */


    //Does not require robot param, may need to add
    public Grabber()
    {
        leftGrabber = new FrcPneumatic(
            "leftGrabber", RobotParams.CANID_PCM, PneumaticsModuleType.CTREPCM,
            RobotParams.PNEUMATIC_LEFT_GRABBER_RETRACT, RobotParams.PNEUMATIC_LEFT_GRABBER_EXTEND);
        rightGrabber = new FrcPneumatic(
            "rightGrabber", RobotParams.CANID_PCM, PneumaticsModuleType.CTREPCM,
            RobotParams.PNEUMATIC_RIGHT_GRABBER_RETRACT, RobotParams.PNEUMATIC_RIGHT_GRABBER_EXTEND);
        release();
        if (RobotParams.Preferences.useVacuum) {
            vacuum = new FrcPWMTalonSRX("vacuum", 1, null, null, null);
        } else {
            vacuum = null;
        }
    }   //Grabber

    //This method is called to grab a cube, extends the left pneumatic for a partial grab
    public void grabCube()
    {   
        if(vacuum != null) {
            if(vacuum.getMotorVelocity() == 0) { //this probably works
                leftGrabber.extend();
            }
        } else {
            leftGrabber.extend();
        }
    }   //grabCube

    //This method is called to grab a cone, extends both pneumatics for a complete grab
    public void grabCone()
    {
        if (vacuum != null) {
            if(vacuum.getMotorVelocity() == 0) { //this probably works
                leftGrabber.extend();
                rightGrabber.extend();
            }
        } else {
            leftGrabber.extend();
            rightGrabber.extend();
        }
    }   //grabCone

    //This method is called to open the grabber, retracting the pneumatics
    public void release()
    {
        leftGrabber.retract();
        rightGrabber.retract();
    }   //release

    //This method is called to turn on the vacuum, used to grab a cube
    public void vacuumOn() {
        if(!leftGrabber.isExtended() && vacuum != null) {
            vacuum.set(1); //this is just a placeholder, unsure of vacuum strength
        }
    }   //vacuumOn

    //This method is called to turn off the vacuum, dropping any grabbed cube
    public void vacuumOff() {
        if (vacuum != null) {
            vacuum.stopMotor();
        }
    }   //vacuumOff

}   //class Grabber