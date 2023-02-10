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
import TrcCommonLib.trclib.TrcPidActuator;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcPidActuator.Parameters;
import TrcFrcLib.frclib.FrcCANFalcon;
import TrcFrcLib.frclib.FrcDigitalInput;

public class Arm implements TrcExclusiveSubsystem{
    private static final String moduleName = "Arm";
    public final Robot robot;
    public final FrcCANFalcon armMotor;
    public final TrcPidActuator arm;
    private TrcDbgTrace msgTracer = null;
    public final FrcDigitalInput armLowerLimitSwitch;

    public Arm(Robot robot) {
        this.robot = robot;
        armMotor = createArmMotor(moduleName + ".motor", RobotParams.CANID_ARM);

        armLowerLimitSwitch = new FrcDigitalInput(moduleName + ".lowerLimitSwitch",
                RobotParams.DIO_ARM_LOWER_LIMIT_SWITCH);

        Parameters params = new Parameters()
                .setPidParams(RobotParams.ARM_KP, RobotParams.ARM_KI, RobotParams.ARM_KD, RobotParams.ARM_TOLERANCE)
                .setPosRange(RobotParams.ARM_MIN_POS, RobotParams.ARM_MAX_POS)
                .setScaleOffset(RobotParams.ARM_INCHES_PER_COUNT, RobotParams.ARM_OFFSET)
                .setZeroCalibratePower(RobotParams.ARM_CAL_POWER);

        arm = new TrcPidActuator(moduleName + ".actuator", armMotor, null, null, params);
    }

    private FrcCANFalcon createArmMotor(String name, int canID) {
        FrcCANFalcon motor = new FrcCANFalcon(name, canID);
        motor.motor.configFactoryDefault();
        motor.motor.configVoltageCompSaturation(RobotParams.BATTERY_NOMINAL_VOLTAGE);
        motor.motor.enableVoltageCompensation(true);
        motor.motor.setSensorPhase(true);
        motor.setBrakeModeEnabled(true);
        motor.setInverted(RobotParams.ARM_MOTOR_INVERTED);

        return motor;
    } // createLiftMotor

    public void setMsgTracer(TrcDbgTrace tracer) {
        msgTracer = tracer;
    } // setMsgTracer

    public boolean isLowerLimitSwitchActive() {
        return armLowerLimitSwitch.isActive();
    } // isLowerLimitSwitchActive

    //
    // Arm PID Actuator methods.
    //

    public void setPower(double power) {
        arm.setPower(power);
    } // setPower

    public void setPidPower(double power) {
        arm.setPidPower(power);
    } // setPidPower

    public void raiseArm() {
        this.setPosition(RobotParams.ARM_RAISED);
    } // setPosition

    public void lowerArm() {
        this.setPosition(RobotParams.ARM_LOWERED);
    } // setPosition

    public void setPosition(double position) {
        arm.setTarget(position);
    } // setPosition

    public void zeroCalibrateArm() {
        final String funcName = "zeroArm";

        if (msgTracer != null) {
            msgTracer.traceInfo(
                    funcName, "[%.3f] RetractArm: currPos=%.1f", TrcTimer.getModeElapsedTime(), arm.getPosition());
        }

        arm.zeroCalibrate();
    }

}
