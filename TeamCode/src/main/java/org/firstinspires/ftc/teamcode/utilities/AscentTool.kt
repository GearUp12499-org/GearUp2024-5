package org.firstinspires.ftc.teamcode.utilities

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.aether.collaborative_multitasking.MultitaskScheduler
import org.firstinspires.ftc.teamcode.Hardware
import org.firstinspires.ftc.teamcode.hardware.HSlideProxy

@TeleOp(name = "util: Lv. 2 Ascent", group = "Utilities")
class AscentTool : LinearOpMode() {
    val tele = telemetry

    override fun runOpMode() {
        val hardware = Hardware(hardwareMap)
        val scheduler = MultitaskScheduler()
        hardware.sharedHardwareInit()
        hardware.arm.position = Hardware.ARM_PRE_WALL_PICK
        val hsp = HSlideProxy(scheduler, hardware, HSlideProxy.Position.OUT)

        waitForStart()

        while (opModeIsActive()) {
            hardware.rightAscent.power =
                if (gamepad1.y) 1.0 // up (towards vertical)
                else if (gamepad1.a) -1.0 // down (towards folded)
                else 0.0
            hardware.leftAscent.power =
                if (gamepad1.dpad_up) 1.0 // down (towards folded)
                else if (gamepad1.dpad_down) -1.0 // up (towards vertical)
                else 0.0

            tele.addData("left ascent encoder", hardware.leftAscentEnc.getCurrentPosition())
            tele.addData("right ascent encoder", hardware.rightAscentEnc.getCurrentPosition())
            tele.update()
            scheduler.tick()
        }
    }
}