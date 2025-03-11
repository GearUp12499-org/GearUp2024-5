@file:Suppress("NOTHING_TO_INLINE")

package org.firstinspires.ftc.teamcode.limelight

import android.util.Log
import dev.aether.collaborative_multitasking.OneShot
import dev.aether.collaborative_multitasking.Scheduler
import dev.aether.collaborative_multitasking.TaskGroup
import dev.aether.collaborative_multitasking.TaskTemplate
import dev.aether.collaborative_multitasking.ext.Pause
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Hardware
import org.firstinspires.ftc.teamcode.hardware.HClawProxy
import org.firstinspires.ftc.teamcode.hardware.HSlideProxy
import kotlin.math.abs
import kotlin.math.roundToInt

@Suppress("unused")
private inline operator fun DoubleArray.component6() = get(5)

@Suppress("unused")
private inline operator fun DoubleArray.component7() = get(6)

@Suppress("unused")
private inline operator fun DoubleArray.component8() = get(7)

@Suppress("unused")
private inline operator fun DoubleArray.component9() = get(8)


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

class LimelightPickupImmediate constructor(
    scheduler: Scheduler,
    private val hardware: Hardware,
    private val hSlideProxy: HSlideProxy,
    private val hClawProxy: HClawProxy,
    private val enabled: Int,
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
                .then(wait(0.100))
                .then(hClawProxy.aSetFlip(Hardware.FLIP_DOWN))
                .then(wait(0.100))
                .then(run { clawTwist.position = Hardware.CLAW_TWIST_INIT })
                .then(hClawProxy.aSetFlipClaw(Hardware.FLIP_ONE_THIRD, Hardware.FRONT_CLOSE))
                .then(groupOf {
                    add(hSlideProxy.moveTransfer())
                    add(wait(0.350))
                        .then(hClawProxy.aSetClaw(Hardware.FRONT_CLOSE_HARD));
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

class LimelightSearch @JvmOverloads constructor(
    scheduler: Scheduler,
    private val hardware: Hardware,
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
        Hardware.Locks.Signals,
        Hardware.Locks.Limelight
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

    var isRecognized = false; private set
    private var liveAngle = 0.0
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

        isRecognized = statusT == ResultStatus.PICKUP
        val color = when (statusT) {
            ResultStatus.PICKUP -> Hardware.LAMP_GREEN
            ResultStatus.MATCH_NO_PICKUP -> Hardware.LAMP_ORANGE
            else -> 0.0
        }
        liveAngle = angle
        liveStatus = statusT
        colorLeft.position = color
        colorRight.position = color
    }

    fun proceed(): Boolean {
        when (liveStatus) {
            ResultStatus.PICKUP -> scheduler.add(
                LimelightPickupImmediate(
                    scheduler,
                    hardware,
                    hSlideProxy,
                    hClawProxy,
                    enabled,
                    liveAngle
                )
            )

            ResultStatus.MATCH_NO_PICKUP -> /* not implemented */ {}
            else -> return false
        }
        done = true
        return true
    }

    override fun invokeIsCompleted() = done

    override fun invokeOnFinish() {
        lightRight.position = 0.0
        lightLeft.position = 0.0
        lightRight.position = 0.0
        limelight.stop()
    }
}