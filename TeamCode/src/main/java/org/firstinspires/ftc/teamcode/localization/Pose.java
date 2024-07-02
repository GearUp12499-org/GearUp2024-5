package org.firstinspires.ftc.teamcode.localization;

/* Kotlin: KPose.kt (data class) */
public class Pose {
    private final double x;
    private final double y;
    private final double heading;

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    public double heading() {
        return heading;
    }

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose(double x, double y) {
        this(x, y, 0);
    }

    /**
     * vector addition (i.e. [x1, y1, heading1] + [x2, y2, heading2] = [x1 + x2, y1 + y2, heading1 + heading2])
     */
    public Pose add(Pose other) {
        return new Pose(x + other.x, y + other.y, heading + other.heading);
    }
}
