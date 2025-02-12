package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp
public class LLlightTeleOp extends LinearOpMode {
    private Hardware hardware;

    @Override
    public void runOpMode() {
        hardware = new Hardware(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            hardware.limelightlight.setPosition(1);
        }
    }
}
