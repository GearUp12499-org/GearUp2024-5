package org.firstinspires.ftc.teamcode.mmooover

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.aether.collaborative_multitasking.MultitaskScheduler
import org.firstinspires.ftc.teamcode.Hardware
import org.firstinspires.ftc.teamcode.mmooover.tasks.MoveToTask
import org.firstinspires.ftc.teamcode.utilities.LoopStopwatch
import org.firstinspires.ftc.teamcode.utilities.degrees


@TeleOp
class Square : LinearOpMode() {
    companion object {
        const val X = 24
    }

    private lateinit var scheduler: MultitaskScheduler
    private lateinit var hardware: Hardware
    private lateinit var tracker: EncoderTracking
    private lateinit var loopTimer: LoopStopwatch
    private lateinit var speed2Power: Speed2Power
    private lateinit var ramps: Ramps
    private lateinit var mMoverDataPack: MMoverDataPack

    private fun hwInit() {
        hardware = Hardware(hardwareMap)
        hardware.sharedHardwareInit()
        tracker = EncoderTracking(hardware)
        loopTimer = LoopStopwatch()
        speed2Power = Speed2Power(0.2)
        ramps = Ramps(
            Ramps.linear(5.0), // t seconds
            Ramps.linear(1 / 3.0), // inches from target
//                Easing.power(3.0, 12.0),
            Ramps.LimitMode.SCALE
        )
        mMoverDataPack = MMoverDataPack(
            hardware, tracker, loopTimer, speed2Power, ramps
        )
    }

    private fun moveTo(target: Pose) = MoveToTask(
        scheduler, mMoverDataPack, target, telemetry
    )

    override fun runOpMode() {
        scheduler = MultitaskScheduler()
        hwInit()
        scheduler.add(moveTo(Pose(X, 0, 0)))
            .then(moveTo(Pose(X, X, 0)))
            .then(moveTo(Pose(0, X, 0)))
            .then(moveTo(Pose(0, 0, 0)))
            .then(moveTo(Pose(X, 0, 90.degrees)))
            .then(moveTo(Pose(X, X, 180.degrees)))
            .then(moveTo(Pose(0, X, 270.degrees)))
            .then(moveTo(Pose(0, 0, 360.degrees)))

        waitForStart()

        while (opModeIsActive()) {
            loopTimer.click()
            tracker.step()
            scheduler.tick()
            scheduler.displayStatus(true, true, telemetry::addLine)
            telemetry.update()
        }
    }
}