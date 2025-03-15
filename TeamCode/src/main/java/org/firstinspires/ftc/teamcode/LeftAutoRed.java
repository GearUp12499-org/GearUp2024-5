package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class LeftAutoRed extends LeftAuto {
    @Override
    boolean isRed() {
        return true;
    }
}
