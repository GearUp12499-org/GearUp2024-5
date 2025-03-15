@file:Suppress("NOTHING_TO_INLINE")

package org.firstinspires.ftc.teamcode.limelight

import android.util.Log
import com.qualcomm.robotcore.util.ElapsedTime
import dev.aether.collaborative_multitasking.ITask
import dev.aether.collaborative_multitasking.OneShot
import dev.aether.collaborative_multitasking.Scheduler
import dev.aether.collaborative_multitasking.TaskGroup
import dev.aether.collaborative_multitasking.TaskTemplate
import dev.aether.collaborative_multitasking.ext.Pause
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Hardware
import org.firstinspires.ftc.teamcode.Hardware.Locks
import org.firstinspires.ftc.teamcode.hardware.HClawProxy
import org.firstinspires.ftc.teamcode.hardware.HSlideProxy
import org.firstinspires.ftc.teamcode.limelight.LimelightSearch.ResultStatus
import org.firstinspires.ftc.teamcode.limelight.LimelightSearch.ResultStatus.entries
import org.firstinspires.ftc.teamcode.mmooover.MMoverDataPack
import org.firstinspires.ftc.teamcode.mmooover.Motion
import org.firstinspires.ftc.teamcode.mmooover.Pose
import org.firstinspires.ftc.teamcode.mmooover.tasks.MoveToTask
import kotlin.collections.component1
import kotlin.collections.component2
import kotlin.collections.component3
import kotlin.collections.component4
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.roundToInt
import kotlin.math.sin


object LimelightDetectionMode {
    const val RED = 0b0001
    const val YELLOW = 0b0010
    const val BLUE = 0b0100

    fun toArray(bitset: Int): DoubleArray {
        return doubleArrayOf(
            if ((bitset and YELLOW) > 0) 1.0 else 0.0,
            if ((bitset and RED) > 0) 1.0 else 0.0,
            if ((bitset and BLUE) > 0) 1.0 else 0.0,
        )
    }
}

class LimelightPickupImmediate(
    scheduler: Scheduler,
    private val hardware: Hardware,
    private val hSlideProxy: HSlideProxy,
    private val hClawProxy: HClawProxy,
    private var angle: Double
) : TaskGroup(scheduler) {
    private val depends = setOf(
        hClawProxy.CONTROL_CLAW,
        hClawProxy.CONTROL_FLIP,
    )

    private fun wait(seconds: Number) = Pause(innerScheduler, seconds.toDouble())
    private fun run(lambda: () -> Unit) = OneShot(innerScheduler, lambda)

    private fun groupOf(contents: Scheduler.() -> Unit): TaskGroup {
        return TaskGroup(scheduler).with(contents)
    }

    private val lightLeft = hardware.lightLeft
    private val lightRight = hardware.lightRight
    private val clawTwist = hardware.clawTwist
    private val clawColor = hardware.clawColor
    private val pickupPosition: Double
    private val twistDelay: Double

    init {
        if (angle < 90) {
            pickupPosition = Hardware.CLAW_TWIST_INIT - (angle * 0.0037)
        } else {
            angle = 180 - angle
            pickupPosition = Hardware.CLAW_TWIST_INIT + (angle * 0.0037)
        }
        twistDelay =
            (100.0 + (500 / 0.33) * abs(pickupPosition - Hardware.CLAW_TWIST_INIT)) / 1000.0

        extraDeps.addAll(depends)
        this.with {
            it.add(run {
                clawTwist.position = pickupPosition
            })
                .then(hClawProxy.aSetFlip(Hardware.FLIP_DOWN))
                .then(wait(0.200))
                .then(run {
                    hClawProxy.setFlip(Hardware.FLIP_DOWN_PLUS)
                })
                .then(wait(0.100))
                .then(hClawProxy.aSetClaw(Hardware.FRONT_CLOSE_HARD))
                .then(wait(0.150))
                .then(hClawProxy.aSetFlip(Hardware.FLIP_DOWN))
                .then(wait(0.150))
                .then(run { clawTwist.position = Hardware.CLAW_TWIST_INIT })
                .then(hClawProxy.aSetFlipClaw(Hardware.FLIP_ONE_THIRD, Hardware.FRONT_CLOSE))
                .then(groupOf {
                    add(hSlideProxy.moveTransfer())
                    add(wait(0.350))
                        .then(hClawProxy.aSetClaw(Hardware.FRONT_CLOSE_HARD))
                })
                .then(hClawProxy.aSetFlip(Hardware.FLIP_UP))
        }
    }

    override fun invokeOnStart() {
        super.invokeOnStart()
        lightLeft.position = 0.0
        lightRight.position = 0.0
    }
}

class LimelightMoveAdjust(
    scheduler: Scheduler,
    private val hardware: Hardware,
    private val mmoverData: MMoverDataPack,
    x: Number,
    y: Number,
    angle: Number,
    private val hSlideProxy: HSlideProxy,
    private val hClawProxy: HClawProxy,
    private val telemetry: Telemetry?
) : TaskGroup(scheduler) {
    private val limelight = hardware.limelight
    private val colorLeft = hardware.colorLeft
    private val colorRight = hardware.colorRight

    private val motors = hardware.driveMotors

    private lateinit var targetPose: Pose
    private var forward = -y.toDouble()
    private var timerLimit = hypot(x.toDouble(), y.toDouble()) * .50 /* seconds per inch */
    private var right = x.toDouble()
    private var angle = angle.toDouble()

    private val depends = setOf(
        Locks.DriveMotors,
        Locks.Signals,
        Locks.Limelight
    )

    private fun data(caption: String, value: Any?) = when {
        telemetry != null -> telemetry.addData(caption, value)
        else -> null
    }

    private fun line(caption: String) = when {
        telemetry != null -> telemetry.addLine(caption)
        else -> null
    }

    override fun requirements() = depends

    override fun invokeOnStart() {
        colorLeft.position = Hardware.LAMP_PURPLE
        colorRight.position = Hardware.LAMP_PURPLE
        targetPose = mmoverData.tracking.getPose() + Motion(forward, right, 0)
        this.with {
            val t = MoveToTask(scheduler, mmoverData, targetPose, telemetry)
            t.acceptDist = 0.25
            it.add(t)
        }
    }

    private operator fun Pose.plus(value: Motion) = Pose(
        this.x + value.forward * cos(this.heading) + value.right * sin(this.heading),
        this.y + value.forward * sin(this.heading) - value.right * cos(this.heading),
        this.heading
    )

    override fun invokeOnFinish() {
        super.invokeOnFinish()
        scheduler.add(
            LimelightPickupImmediate(
                scheduler, hardware, hSlideProxy, hClawProxy, angle
            )
        )
        colorLeft.position = 0.0
        colorRight.position = 0.0
        motors.setAll(0)
        limelight.stop()
    }
}

open class LimelightSearch @JvmOverloads constructor(
    scheduler: Scheduler,
    private val hardware: Hardware,
    private val mmoverData: MMoverDataPack,
    private val hSlideProxy: HSlideProxy,
    private val hClawProxy: HClawProxy,
    private var enabled: Int,
    private val telemetry: Telemetry? = null,
) :
    TaskTemplate(scheduler) {

    enum class ResultStatus(val intValue: Int?, val message: String) {
        ERROR_RAISED(-1, "Catastrophic failure (Pipeline raised error)"),
        NO_MATCH(0, "No samples detected"),
        MATCH_NO_PICKUP(1, "Sample detected, but can't grab"),
        PICKUP(2, "Sample detected in grab range"),
        ELSE(-2, "Catastrophic failure (Pipeline returned invalid status code)");

        companion object {
            fun getForId(id: Int) = entries.firstOrNull { it.intValue == id } ?: ELSE
        }
    }

    companion object {
        private var lastEnabledState = -1
    }

    private val limelight = hardware.limelight

    //private val lightRight = hardware.lightRight
    private val lightLeft = hardware.lightLeft
    private val lightRight = hardware.lightRight
    private val colorLeft = hardware.colorLeft
    private val colorRight = hardware.colorRight

    private val depends = setOf(
        hSlideProxy.CONTROL,
        hClawProxy.CONTROL_CLAW,
        hClawProxy.CONTROL_FLIP,
        Locks.Signals,
        Locks.Limelight
    )

    override fun requirements() = depends

    private var done = false

    private fun data(caption: String, value: Any?) = when {
        telemetry != null -> telemetry.addData(caption, value)
        else -> null
    }

    private fun line(caption: String) = when {
        telemetry != null -> telemetry.addLine(caption)
        else -> null
    }

    override fun invokeOnStart() {
        if (lastEnabledState != enabled) {
            limelight.updatePythonInputs(LimelightDetectionMode.toArray(enabled))
            lastEnabledState = enabled
        }

        limelight.start()
        lightLeft.position = 1.0
        lightRight.position = 1.0 // white
        hSlideProxy.moveOutSync()
        hClawProxy.setClaw(Hardware.FRONT_OPEN)
    }

    private var liveAngle = 0.0
    private var liveXOff = 0.0
    private var liveYOff = 0.0
    private var liveStatus = ResultStatus.NO_MATCH

    override fun invokeOnTick() {
        val result = limelight.latestResult
        if (result == null) {
            data("Limelight", "Result is null")
            return
        }
        data("LL Pipeline No", limelight.status.pipelineIndex)
        val pyOut = result.pythonOutput
        val (status, xOff, yOff, angle) = pyOut
        val statusT = ResultStatus.getForId(status.roundToInt())
        data("LL: status", statusT.message)
        data("LL: x", xOff)
        data("LL: y", yOff)
        data("LL: angle", angle)

        val color = when (statusT) {
            ResultStatus.PICKUP -> Hardware.LAMP_GREEN
            ResultStatus.MATCH_NO_PICKUP -> Hardware.LAMP_ORANGE
            else -> 0.0
        }
        liveAngle = angle
        liveStatus = statusT
        liveXOff = xOff
        liveYOff = yOff
        colorLeft.position = color
        colorRight.position = color
    }

    fun proceed(): Boolean {
        when (liveStatus) {
            ResultStatus.PICKUP -> {
                scheduler.add(
                    LimelightPickupImmediate(
                        scheduler,
                        hardware,
                        hSlideProxy,
                        hClawProxy,
                        liveAngle
                    )
                )
                lightRight.position = 0.0
                limelight.stop()
            }

            ResultStatus.MATCH_NO_PICKUP -> scheduler.add(
                LimelightMoveAdjust(
                    scheduler,
                    hardware,
                    mmoverData,
                    liveXOff,
                    liveYOff,
                    liveAngle,
                    hSlideProxy,
                    hClawProxy,
                    telemetry
                )
            )

            else -> return false
        }
        done = true
        return true
    }

    override fun invokeIsCompleted() = done

    override fun invokeOnFinish() {
        lightRight.position = 0.0
        lightLeft.position = 0.0
    }
}

class LimelightAuto(
    scheduler: Scheduler, hardware: Hardware, mmoverData: MMoverDataPack,
    hSlideProxy: HSlideProxy, hClawProxy: HClawProxy, enabled: Int, maxDuration: Number
) : TaskGroup(scheduler) {
    private val maxDuration = maxDuration.toDouble()
    private val runtime = ElapsedTime()

    private val limelightSearchTask: LimelightSearch =
        LimelightSearch(innerScheduler, hardware, mmoverData, hSlideProxy, hClawProxy, enabled)

    init {
        with {
            it.add(OneShot(it) {
                hClawProxy.setFlipClaw(Hardware.FLIP_UP, Hardware.FRONT_OPEN)
            })
            it.add(limelightSearchTask)
        }
    }

    override fun invokeOnStart() {
        super.invokeOnStart()
        runtime.reset()
    }

    override fun invokeOnTick() {
        super.invokeOnTick()
        // just spam the thing idfk
        if (limelightSearchTask.state == ITask.State.Ticking) limelightSearchTask.proceed()
        if (runtime.time() > maxDuration) requestStop()
    }
}