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

import TrcCommonLib.trclib.TrcDbgTrace; 
import TrcCommonLib.trclib.TrcExclusiveSubsystem; 
import TrcCommonLib.trclib.TrcTimer; 
import TrcFrcLib.frclib.FrcCANFalcon; 
import TrcFrcLib.frclib.FrcPneumatic; 
import edu.wpi.first.wpilibj.PneumaticsModuleType; 
//package and imports

public class Intake implements TrcExclusiveSubsystem { 
    private static final String moduleName = "Intake"; 
    private final FrcCANFalcon intakeMotorRight; 
    private final FrcCANFalcon intakeMotorLeft; 
    private final FrcPneumatic intakePneumatic; 
    private TrcDbgTrace msgTracer = null; 

    public Intake(TrcDbgTrace msgTracer) { 
        this.msgTracer = msgTracer;

        intakeMotorRight = new FrcCANFalcon(moduleName + ".rightMotor", RobotParams.CANID_INTAKE_RIGHT); 
        intakeMotorRight.motor.configFactoryDefault(); 
        intakeMotorLeft = new FrcCANFalcon(moduleName + ".leftMotor", RobotParams.CANID_INTAKE_LEFT); 
        intakeMotorLeft.motor.configFactoryDefault(); 
        intakePneumatic = new FrcPneumatic( moduleName + ".pneumatic", RobotParams.CANID_PCM, PneumaticsModuleType.CTREPCM, 
            RobotParams.PNEUMATIC_INTAKE_RETRACT, RobotParams.PNEUMATIC_INTAKE_EXTEND); 
        intakePneumatic.retract(); 
        //intake motors and pneumatics
    } 

    public void cancel() { 
        intakeMotorRight.set(0.0); 
        intakeMotorLeft.set(0.0); 
    } //cancel out the right and left intakes

    public double getMotorPower()  { 
        return intakeMotorRight.getMotorPower(); 
    } //get the motor power 

  
    public void setPower(String owner, double delay, double power, double duration) { 
        final String funcName = "setPower"; 
        if (msgTracer != null) { 
            msgTracer.traceInfo( 
            funcName, "[%.3f] owner=%s, delay=%.1f, power=%.1f, duration=%.3f", 
            TrcTimer.getModeElapsedTime(), owner, delay, power, duration); 
        } 

        if (validateOwnership(owner)) { 
            intakeMotorRight.set(delay, power, duration); 
            intakeMotorLeft.set(delay, power, duration); 
        } //validate ownership

    }   

    public void setPower(double delay, double power, double duration) { 
        setPower(null, delay, power, duration); 
    }   //setPower 

    public void setPower(double power) { 
        setPower(null, 0.0, power, 0.0); 
    }   //setPower 

} 