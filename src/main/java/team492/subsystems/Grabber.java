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

package team492.subsystems;

import java.util.Locale;

import TrcCommonLib.trclib.TrcTimer;
import TrcFrcLib.frclib.FrcPWMTalonSRX;
import TrcFrcLib.frclib.FrcPneumatic;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import team492.RobotParams;

public class Grabber
{
    private static final String moduleName = "Grabber";
    private final FrcPneumatic coneGrabber;
    private final FrcPneumatic cubeGrabber;
    private final TrcTimer timer;
    private final FrcPWMTalonSRX vacuum;
    
    /**
     * Constructor: Create an instance of the object.
     *
     */


    //Does not require robot param, may need to add
    public Grabber()
    {
        coneGrabber = new FrcPneumatic(
            moduleName + ".cone", RobotParams.CANID_PCM, PneumaticsModuleType.REVPH,
            RobotParams.PNEUMATIC_CONE_GRABBER_RETRACT, RobotParams.PNEUMATIC_CONE_GRABBER_EXTEND);
        cubeGrabber = new FrcPneumatic(
            moduleName + ".cube", RobotParams.CANID_PCM, PneumaticsModuleType.REVPH,
            RobotParams.PNEUMATIC_CUBE_GRABBER_RETRACT, RobotParams.PNEUMATIC_CUBE_GRABBER_EXTEND);
        timer = new TrcTimer(moduleName);

        if (RobotParams.Preferences.useVacuum)
        {
            vacuum = new FrcPWMTalonSRX("vacuum", RobotParams.PWM_VACUUM, null, null, null);
        }
        else
        {
            vacuum = null;
        }
    }   //Grabber

    /**
     * This method returns the state of the Arm in a string.
     */
    @Override
    public String toString()
    {
        return String.format(
            Locale.US, "%s: grabbedCube=%s, grabbedCone=%s", moduleName, grabbedCube(), grabbedCone());
    }   //toString

    //This method is called to grab a cube, extends the left pneumatic for a partial grab
    public void grabCube()
    {   
        if (vacuum == null)
        {
            cubeGrabber.extend();
        }
        else if (vacuum.getMotorVelocity() == 0)
        {
            //this probably works
            cubeGrabber.extend();
        }
    }   //grabCube

    public void releaseCube()
    {
        cubeGrabber.retract();
    }   //releaseCube

    //This method is called to grab a cone, extends both pneumatics for a complete grab
    public void grabCone()
    {
        if (vacuum == null)
        {
            coneGrabber.extend();
        }
        else if (vacuum.getMotorVelocity() == 0)
        {
            //this probably works
            coneGrabber.extend();
        }
    }   //grabCone

    public void releaseCone(double delay)
    {
        if (delay > 0.0)
        {
            timer.set(delay, this::delayReleaseCone);
        }
        else
        {
            coneGrabber.retract();
        }
    }   //releaseCone

    public void releaseCone()
    {
        releaseCone(0.0);
    }   //releaseCone

    private void delayReleaseCone(Object context)
    {
        coneGrabber.retract();
    }   //delayReleaseCone

    //This method is called to open the grabber, retracting the pneumatics
    public void releaseAll()
    {
        coneGrabber.retract();
        cubeGrabber.retract();
    }   //releaseAll

    public boolean grabbedCube()
    {
        return cubeGrabber.isExtended();
    }   //grabbedCube

    public boolean grabbedCone()
    {
        return coneGrabber.isExtended();
    }   //grabbedCone

    //This method is called to turn on the vacuum, used to grab a cube
    public void vacuumOn()
    {
        if (vacuum != null && !cubeGrabber.isExtended() &&  !coneGrabber.isExtended())
        {
            vacuum.set(1); //this is just a placeholder, unsure of vacuum strength
        }
    }   //vacuumOn

    //This method is called to turn off the vacuum, dropping any grabbed cube
    public void vacuumOff()
    {
        if (vacuum != null)
        {
            vacuum.stopMotor();
        }
    }   //vacuumOff

}   //class Grabber
