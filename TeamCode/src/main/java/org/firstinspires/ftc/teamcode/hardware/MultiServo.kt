package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.Servo

class MultiServo(val primary: Servo, val secondary: Servo, val basis: Double) : Servo {
    override fun getController() = primary.controller

    override fun getPortNumber() = primary.portNumber

    override fun setDirection(direction: Servo.Direction?) {
        primary.direction = direction
        secondary.direction = when (direction) {
            Servo.Direction.FORWARD -> Servo.Direction.REVERSE
            Servo.Direction.REVERSE -> Servo.Direction.FORWARD
            null -> null
        }
    }

    override fun getDirection() = primary.direction

    override fun setPosition(position: Double) {
        primary.position = position
        secondary.position = basis - position
    }

    override fun getPosition() = primary.position

    override fun scaleRange(min: Double, max: Double) = when {
        min == 0.0 && max == 1.0 -> {}
        else -> throw NotImplementedError("Not available on this servo interface")
    }

    override fun getManufacturer() = primary.manufacturer

    override fun getDeviceName() = "FrontFlip proxy object"

    override fun getConnectionInfo() = "${primary.connectionInfo} / ${secondary.connectionInfo}"

    override fun getVersion() = (primary.version and 0xffff) or (secondary.version and 0xffff shl 16)

    override fun resetDeviceConfigurationForOpMode() {
        primary.resetDeviceConfigurationForOpMode()
        secondary.resetDeviceConfigurationForOpMode()
    }

    override fun close() {
        primary.close()
        secondary.close()
    }
}