
package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class revblinkintasteful {
    RevBlinkinLedDriver lights;

    public revblinkintasteful(HardwareMap hardwareMap){ //Run this in Int to map the class items
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
    }

    public void Update_Lights(boolean possession,boolean Rotation,boolean A){
        if (possession){ //The possession is if we have freight in our robot
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        else if (Rotation){ //The Rotation is if the turn table motor is on
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST);
        }
        else if (A){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        else if (!A){
            lights/*THIS IS TECHNICALLY ALLOWED HAHA*/.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }
    }


}
