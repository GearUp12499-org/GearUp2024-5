package org.firstinspires.ftc.teamcode.mmooover

import org.firstinspires.ftc.teamcode.Hardware
import org.firstinspires.ftc.teamcode.utilities.LoopStopwatch

data class MMoverDataPack(
    @JvmField val hardware: Hardware,
    @JvmField val tracking: EncoderTracking,
    @JvmField val loopTimer: LoopStopwatch,
    @JvmField val speed2Power: Speed2Power,
    @JvmField val ramps: Ramps,
)
