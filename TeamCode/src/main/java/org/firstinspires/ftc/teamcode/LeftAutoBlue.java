package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class LeftAutoBlue extends LeftAuto {
    @Override
    boolean isRed() {
        return false;
    }
}
