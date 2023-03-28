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


 import java.util.concurrent.atomic.AtomicBoolean;
 
 import TrcCommonLib.trclib.TrcDbgTrace;
 import TrcCommonLib.trclib.TrcTriggerDigitalInput;
 import TrcCommonLib.trclib.TrcEvent;
 import TrcCommonLib.trclib.TrcExclusiveSubsystem;
 import TrcCommonLib.trclib.TrcTimer;
 import TrcFrcLib.frclib.FrcCANSparkMax;
import TrcFrcLib.frclib.FrcDigitalInput;
import team492.RobotParams;


public class newIntake implements TrcExclusiveSubsystem {

    private static final String moduleName = "newIntake";

    // Init motors and such here
    private final LEDIndicator ledIndicator;
    private final TrcDbgTrace msgTracer;

    private final FrcCANSparkMax intakeMotor;

    private final FrcDigitalInput intakeSensor;
    private final TrcTriggerDigitalInput intakeTrigger;
    private boolean sensorActive = false;
    private TrcEvent triggerEvent = null;  




    public newIntake(LEDIndicator ledIndicator, TrcDbgTrace msgTracer){

        this.ledIndicator = ledIndicator;
        this.msgTracer = msgTracer;

        intakeMotor = new FrcCANSparkMax("intakeMotor", RobotParams.CANID_INTAKE, true);

        intakeSensor = new FrcDigitalInput("intakeSensor", RobotParams.DIO_INTAKE_SENSOR);
        intakeSensor.setInverted(true);
        intakeTrigger = new TrcTriggerDigitalInput("intakeTrigger", intakeSensor);

    }

    public double MotorPower()
    {
        return intakeMotor.getMotorPower();
    }

    
    public void setPower(String owner, double delay, double Power, double duration)
    {
        final String funcName = "setPower";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(
            funcName, "[%.3f] owner=%s, delay=%.1f, Power=%.1f, duration=%.3f",
            TrcTimer.getModeElapsedTime(), owner, delay, Power, duration);
        }

        if (validateOwnership(owner))
        {
            intakeMotor.set(delay, Power, duration);

        }
    }   //setPower



    public void setPower(double delay, double Power, double duration)
    {
        setPower(null, delay, Power, duration);
    }   //setPower




    public boolean hasObject()
    {
        return sensorActive;
    }   //hasObject



   public void enableTrigger(TrcEvent event)
   {
       triggerEvent = event;
       if (triggerEvent != null)
       {
           triggerEvent.clear();
       }
       intakeTrigger.enableTrigger(this::intakeTriggerCallback);
   }   //enableTrigger

   /**
    * This method disables the sensor trigger.
    */
   public void disableTrigger()
   {
       if (triggerEvent != null)
       {
           triggerEvent.cancel();
           triggerEvent = null;
       }
       intakeTrigger.disableTrigger();
   }   //disableTrigger

   
   /**
    * This method is called when the intake sensor is triggered.
    *
    * @param context specifies true if an object is captured, false otherwise.
    */
   private void intakeTriggerCallback(Object context)
   {
       final String funcName = "intakeTriggerCallback";
       sensorActive = ((AtomicBoolean) context).get();

       if (triggerEvent != null)
       {
           triggerEvent.signal();
       }

       if (ledIndicator != null)
       {
           ledIndicator.setIntakeHasObject(sensorActive);
       }

       if (msgTracer != null)
       {
           msgTracer.traceInfo(funcName, "[%.3f] active=%s", TrcTimer.getModeElapsedTime(), sensorActive);
       }
   }   //intakeTriggerCallback





    
}
