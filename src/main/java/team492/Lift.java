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

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcExclusiveSubsystem;
import TrcCommonLib.trclib.TrcPidActuator;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcPidActuator.Parameters;
import TrcFrcLib.frclib.FrcCANFalcon;
import TrcFrcLib.frclib.FrcDigitalInput;
import TrcFrcLib.frclib.FrcPneumatic;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Lift implements TrcExclusiveSubsystem{
    private static final String moduleName = "Lift";
    public final Robot robot;
    public final FrcCANFalcon liftMotor;
    public final FrcDigitalInput liftLowerLimitSwitch;
    public final FrcPneumatic liftPneumatic;
    public final TrcPidActuator lift;
    private TrcDbgTrace msgTracer = null;
    // public final TrcDigitalInputTrigger limitSwitchTrigger;
    // public final TrcEvent event;
    // public final TrcTimer timer;
    // public final TrcStateMachine<T> sm;

    public Lift(Robot robot) {
        this.robot = robot;
        liftMotor = createLiftMotor(moduleName + ".motor", RobotParams.CANID_LIFT);
        liftPneumatic = new FrcPneumatic(moduleName + ".pneumatic", RobotParams.CANID_PCM, PneumaticsModuleType.CTREPCM,
                RobotParams.PNEUMATIC_LIFT_RETRACT, RobotParams.PNEUMATIC_LIFT_EXTEND);
        liftPneumatic.retract();

        liftLowerLimitSwitch = new FrcDigitalInput(moduleName + ".lowerLimitSwitch",
                RobotParams.DIO_LIFT_LOWER_LIMIT_SWITCH);
        Parameters params = new Parameters()
                .setPidParams(RobotParams.LIFT_KP, RobotParams.LIFT_KI, RobotParams.LIFT_KD, RobotParams.LIFT_TOLERANCE)
                .setPosRange(RobotParams.LIFT_MIN_POS, RobotParams.LIFT_MAX_POS)
                .setScaleOffset(RobotParams.LIFT_INCHES_PER_COUNT, RobotParams.LIFT_OFFSET)
                .setZeroCalibratePower(RobotParams.LIFT_CAL_POWER);

        lift = new TrcPidActuator(moduleName + ".actuator", liftMotor, liftLowerLimitSwitch, null, params);

        // limitSwitchTrigger = new TrcDigitalInputTrigger(moduleName + ".limitSWTrigger", liftLowerLimitSwitch,
        //         this::limitSwitchEvent);
        // event = new TrcEvent(moduleName + ".event");
        // timer = new TrcTimer(moduleName + ".timer");
        // sm = new TrcStateMachine<>(moduleName + ".sm");
        // // liftTaskObj = TrcTaskMgr.createTask(moduleName + ".liftTask",
        // // this::autoLiftTask);

    }

    private FrcCANFalcon createLiftMotor(String name, int canID) {
        FrcCANFalcon motor = new FrcCANFalcon(name, canID);
        motor.motor.configFactoryDefault();
        motor.motor.configVoltageCompSaturation(RobotParams.BATTERY_NOMINAL_VOLTAGE);
        motor.motor.enableVoltageCompensation(true);
        motor.motor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyClosed, 10);
        motor.motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 10);
        motor.motor.setSensorPhase(true);
        motor.setBrakeModeEnabled(true);
        motor.setInverted(RobotParams.LIFT_MOTOR_INVERTED);

        return motor;
    } // createLiftMotor

    public void setMsgTracer(TrcDbgTrace tracer) {
        msgTracer = tracer;
    } // setMsgTracer

    public boolean isLowerLimitSwitchActive() {
        return liftLowerLimitSwitch.isActive();
    } // isLowerLimitSwitchActive

    //
    // Lift PID Actuator methods.
    //

    public void setPower(double power) {
        lift.setPower(power);
    } // setPower

    public void setPidPower(double power) {
        lift.setPidPower(power);
    } // setPidPower

    // public void setPosition(double position, boolean hold, TrcEvent event, double
    // timeout)
    // {
    // lift.setTarget(position, hold, event, null, timeout);
    // } //setPosition

    // public void setPosition(double position, boolean hold, TrcEvent event)
    // {
    // lift.setTarget(position, hold, event, null, 0.0);
    // } //setPosition

    public void raiseLift() {
        this.setPosition(RobotParams.LIFT_RAISED);
    } // setPosition

    public void lowerLift() {
        this.setPosition(RobotParams.LIFT_LOWERED);
    } // setPosition

    public void setPosition(double position) {
        lift.setTarget(position);
    } // setPosition

    public void extend() {
        liftPneumatic.extend();
    }

    public void retract() {
        liftPneumatic.retract();
    }

    public void zeroCalibrateLift() {
        final String funcName = "zeroLift";

        if (msgTracer != null) {
            msgTracer.traceInfo(
                    funcName, "[%.3f] RetractLift: currPos=%.1f", TrcTimer.getModeElapsedTime(), lift.getPosition());
        }

        lift.zeroCalibrate();
    } // zeroCalibrateLift

    // private void limitSwitchEvent(Object context) {
    // }

}
