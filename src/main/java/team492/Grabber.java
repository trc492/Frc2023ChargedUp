/*
 * Placeholder comment
 */

package team492;

import TrcFrcLib.frclib.FrcPneumatic;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Grabber {

    //private final Robot robot;

    private final FrcPneumatic grabberPneumatic1;
    private final FrcPneumatic grabberPneumatic2;

    public Grabber() {
        //this.robot = robot;

        grabberPneumatic1 = new FrcPneumatic(
            "Grabber1.pneumatic", RobotParams.CANID_PCM, PneumaticsModuleType.CTREPCM,
            RobotParams.PNEUMATIC_GRABBER1_RETRACT, RobotParams.PNEUMATIC_GRABBER1_EXTEND);
        grabberPneumatic2 = new FrcPneumatic(
            "Grabber2.pneumatic", RobotParams.CANID_PCM, PneumaticsModuleType.CTREPCM,
            RobotParams.PNEUMATIC_GRABBER2_RETRACT, RobotParams.PNEUMATIC_GRABBER2_EXTEND);
        grabberPneumatic1.retract();
        grabberPneumatic2.retract();
    }

    public void partialGrab() {
        //may need code ensuring it's not already grabbing
        grabberPneumatic1.extend();
    }

    public void partialRelease() {
        //may need code ensuring it's not in full grab/already open
        grabberPneumatic1.retract();
    }

    public void fullGrab() {
        //may need code ensuring it's not already grabbing
        grabberPneumatic1.extend();
        grabberPneumatic2.extend();
    }

    public void fullRelease() {
        //may need code ensuring it's not in partial grab/already open
        grabberPneumatic1.retract();
        grabberPneumatic2.retract();
    }
}