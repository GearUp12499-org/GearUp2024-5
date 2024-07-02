package org.firstinspires.ftc.teamcode.localization


/* leave private until approved and synced */
@Deprecated("Kotlin implementations are not approved for use yet")
private data class KPose(
    val x: Double,
    val y: Double,
    val heading: Double,
) {
    /**
     * vector addition (i.e. [x1, y1, heading1] + [x2, y2, heading2] = [x1 + x2, y1 + y2, heading1 + heading2])
     */
    @JvmName("add")
    operator fun plus(other: KPose) = KPose(x + other.x, y + other.y, heading + other.heading)

    @JvmOverloads
    constructor(x: Number, y: Number, heading: Number = 0.0) : this(
        x.toDouble(),
        y.toDouble(),
        heading.toDouble()
    )
}