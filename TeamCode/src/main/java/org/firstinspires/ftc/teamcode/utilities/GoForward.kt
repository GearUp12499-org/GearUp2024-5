package org.firstinspires.ftc.teamcode.utilities

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Hardware

@TeleOp
@Disabled
class GoForward : LinearOpMode() {
    override fun runOpMode() {
        val h = Hardware(hardwareMap)
        h.sharedHardwareInit()
        waitForStart()
        h.driveMotors.setAll(0.175)
        while (opModeIsActive());
    }
}