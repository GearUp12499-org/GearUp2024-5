package org.firstinspires.ftc.teamcode.mmooover.tasks

import android.util.Log
import dev.aether.collaborative_multitasking.Scheduler
import dev.aether.collaborative_multitasking.SharedResource
import dev.aether.collaborative_multitasking.TaskTemplate
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.Hardware
import org.firstinspires.ftc.teamcode.mmooover.Speed2Power
import kotlin.math.abs

class WallApproachTask(
    scheduler: Scheduler,
    hardware: Hardware,
    val target: Double,
    val turnTarget: Double,
    val speed2power: Speed2Power,
) : TaskTemplate(scheduler) {
    companion object {
        val requirements = setOf(Hardware.Locks.DriveMotors)

        const val TURN_KP = 0.9 / 60
        const val FWD_KP = 0.1
    }

    val gyro = hardware.gyro
    val distanceLeft = hardware.distanceFrontLeft
    val distanceRight = hardware.distanceFrontRight
    val frontLeft = hardware.frontLeft
    val frontRight = hardware.frontRight
    val backLeft = hardware.backLeft
    val backRight = hardware.backRight

    var error: Double = Double.MAX_VALUE

    override fun requirements(): Set<SharedResource> = requirements

    private fun clamp(a: Double, low: Double, high: Double) = when {
        a < low -> low
        a > high -> high
        else -> a
    }

    override fun invokeOnTick() {
        val currentAngles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
        val heading = currentAngles.firstAngle
        val turnError = heading - turnTarget
        val delta = turnError * TURN_KP

        val lDist = distanceLeft.getDistance(DistanceUnit.INCH)
        val rDist = distanceRight.getDistance(DistanceUnit.INCH)
        val avgDist = (lDist + rDist) / 2.0
        error = abs(avgDist - target)
        val power = clamp(error * FWD_KP - delta, -0.3, 0.3)
        val powerB = clamp(error * FWD_KP + delta, -0.3, 0.3)
        frontRight.power = power
        frontLeft.power = powerB
        backRight.power = power
        backLeft.power = powerB

        Log.i("WallApproachTask", "left/right: %.4f %.4f inch; angle: %.4f deg; power: %.2f (%.2f turn)".format(lDist, rDist, heading, power, powerB))
    }

    override fun invokeIsCompleted(): Boolean {
        return error < 0.6
    }
}