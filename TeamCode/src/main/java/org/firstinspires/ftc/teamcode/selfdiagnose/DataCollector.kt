@file:SuppressLint("DefaultLocale")

package org.firstinspires.ftc.teamcode.selfdiagnose

import android.annotation.SuppressLint
import android.util.Log
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.EncoderTrackingTest
import org.firstinspires.ftc.teamcode.Hardware
import org.firstinspires.ftc.teamcode.mmooover.EncoderTracking
import java.lang.String.format
import kotlin.math.PI
import kotlin.math.abs

@TeleOp
class DataCollector : LinearOpMode() {
    companion object {
        /** CONFIGURATION **/
        const val INITIAL_POWER = 1.0
        const val POWER_STEP = -0.1

        /**
         * Motor "on" time; total time the motor will be set
         */
        const val TIME_RUN = .5 // seconds

        /**
         * "Warm-up" time; data collection starts after this time period is over
         */
        const val TIME_WARMUP = .2 // seconds

        /**
         * Motor "off" time between data collections to allow the robot to come to a stop
         */
        const val TIME_WAIT = .5

        /** STATIC ITEMS **/
        @JvmStatic
        fun now() = System.nanoTime() / 1.0e9
    }
    private fun tick2inch(ticks: Number): Double {
        return (ticks.toDouble() / hardware.encoderTicksPerRevolution) * 2 * PI * hardware.encoderWheelRadius
    }

    enum class State {
        RUN_WARMUP,
        RUN_MEASURING,
        WAITING
    }

    open class StepInfo(
        private val timeThisState: Double,
        private val leftTicks: Int,
        private val centerTicks: Int,
        private val rightTicks: Int,
        private val state: String
    ) {
        private val timestamp = System.currentTimeMillis() / 1000.0

        open fun getParamPack() = listOf(
            format("\"timeThisState\": %.6f", timeThisState),
            "\"leftTicks\": $leftTicks",
            "\"centerTicks\": $centerTicks",
            "\"rightTicks\": $rightTicks",
            "\"state\": \"$state\""
        )

        @SuppressLint("DefaultLocale")
        override fun toString(): String {
            val inner = getParamPack().joinToString(",")
            return String.format(
                "##%.3f##{$inner}##",
                timestamp,
            )
        }
    }

    open class WaitingStepInfo(
        timeThisState: Double,
        leftTicks: Int,
        centerTicks: Int,
        rightTicks: Int
    ) : StepInfo(timeThisState, leftTicks, centerTicks, rightTicks, State.WAITING.name)

    open class RunStepInfo(
        private val power: Double,
        private val direction: Double,
        private val x: Double,
        private val y: Double,
        private val rot: Double,
        timeThisState: Double,
        leftTicks: Int,
        centerTicks: Int,
        rightTicks: Int,
        state: String
    ) : StepInfo(timeThisState, leftTicks, centerTicks, rightTicks, state) {
        override fun getParamPack(): List<String> {
            return super.getParamPack() + listOf(
                format("\"power\": %.3f", power),
                format("\"direction\": %.1f", direction),
                format("\"pose\": [%.6f,%.6f,%.6f]", x, y, rot)
            )
        }
    }

    private fun send(data: StepInfo) {
        Log.d("DataCollector_dump", data.toString())
    }

    private lateinit var hardware: Hardware
    private lateinit var tracker: EncoderTracking
    private var state: State = State.WAITING

    val powerVsSpeed: MutableMap<Double, Double> = mutableMapOf()

    private fun collectWaiting(time: Double) = send(
        WaitingStepInfo(
            time,
            hardware.encoderLeft.currentPosition,
            hardware.encoderCenter.currentPosition,
            hardware.encoderRight.currentPosition
        )
    )

    private fun collectRunning(time: Double, power: Double, direction: Double) {
        val pose = tracker.pose
        send(
            RunStepInfo(
                power, direction,
                pose.x, pose.y, pose.heading,
                time,
                hardware.encoderLeft.currentPosition,
                hardware.encoderCenter.currentPosition,
                hardware.encoderRight.currentPosition,
                state.name
            )
        )
    }

    override fun runOpMode() {
        hardware = Hardware(hardwareMap, true)
        tracker = EncoderTracking(hardware)

        var stateEntryTime: Double
        var startTime = 0.0
        var stopTime = 0.0
        var startPos = 0.0
        var stopPos = 0.0

        var direction = +1.0

        var power = INITIAL_POWER
        waitForStart()

        state = State.RUN_WARMUP
        stateEntryTime = now()
        tracker.clear()

        main@while (opModeIsActive()) {
            val syncNow = now()
            val timeThisState = syncNow - stateEntryTime
            hardware.clearCache()
            tracker.step()
            when (state) {
                State.RUN_WARMUP -> {
                    hardware.driveMotors.setAll(power * direction)
                    if (timeThisState >= TIME_WARMUP) {
                        state = State.RUN_MEASURING
                        startTime = syncNow
                        startPos =
                            tick2inch((hardware.encoderLeft.currentPosition + hardware.encoderRight.currentPosition) / 2.0)
                        continue@main
                    }
                    collectRunning(timeThisState, power, direction)
                }

                State.RUN_MEASURING -> {
                    hardware.driveMotors.setAll(power * direction)
                    if (timeThisState >= TIME_RUN) {
                        state = State.WAITING
                        stopTime = syncNow
                        stopPos =
                            tick2inch((hardware.encoderLeft.currentPosition + hardware.encoderRight.currentPosition) / 2.0)
                        val distanceTravel = (stopPos - startPos)
                        if (abs(distanceTravel) < 1) {
                            Log.w("DataCollector", "aborting")
                            break@main
                        }
                        powerVsSpeed[power] = (stopPos - startPos) / (stopTime - startTime)
                        Log.i("DataCollector", "power $power speed ${(stopPos - startPos) / (stopTime - startTime)} inches/second")
                        stateEntryTime = syncNow
                        continue@main
                    }
                    collectRunning(timeThisState, power, direction)
                }

                State.WAITING -> {
                    hardware.driveMotors.setAll(0.0)
                    if (timeThisState >= TIME_WAIT) {
                        state = State.RUN_WARMUP
                        power += POWER_STEP
                        direction *= -1
                        if (power <= 1e-6) break@main
                        tracker.clear()
                        stateEntryTime = syncNow
                        continue@main
                    }

                    collectWaiting(timeThisState)
                }
            }
            telemetry.addLine("State: ${state.name}")
            telemetry.addLine(format("State Time: %.2fs", stateEntryTime))
            telemetry.update()
        }

        StringBuilder().apply {
            append("{")
            powerVsSpeed.forEach { (k, v) ->
                append(k)
                append(": ")
                append(v)
                append(",")
            }
            append("}")
        }.also {
            Log.i("DataCollector", it.toString())
        }

        spin@while (opModeIsActive()) {
            hardware.clearCache()
            hardware.driveMotors.setAll(0.0)
            telemetry.addLine("done!")
            telemetry.update()
        }
    }
}