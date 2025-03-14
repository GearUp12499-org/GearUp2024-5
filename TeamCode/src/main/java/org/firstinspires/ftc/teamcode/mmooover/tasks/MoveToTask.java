package org.firstinspires.ftc.teamcode.mmooover.tasks;

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

import android.annotation.SuppressLint;
import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.RightAuto;
import org.firstinspires.ftc.teamcode.mmooover.EncoderTracking;
import org.firstinspires.ftc.teamcode.mmooover.MMoverDataPack;
import org.firstinspires.ftc.teamcode.mmooover.Motion;
import org.firstinspires.ftc.teamcode.mmooover.Pose;
import org.firstinspires.ftc.teamcode.mmooover.Ramps;
import org.firstinspires.ftc.teamcode.mmooover.Speed2Power;
import org.firstinspires.ftc.teamcode.utilities.LoopStopwatch;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.Set;

import dev.aether.collaborative_multitasking.Scheduler;
import dev.aether.collaborative_multitasking.SharedResource;
import dev.aether.collaborative_multitasking.TaskTemplate;

@SuppressLint("DefaultLocale")
public class MoveToTask extends TaskTemplate {
    public static final double kD = 0.005;

    public double acceptDist = 1.0;

    protected Pose target;
    protected final EncoderTracking tracker;
    protected final Speed2Power speed2Power;
    protected final Ramps ramps;
    protected final @Nullable Telemetry telemetry;
    protected final LoopStopwatch loopTimer;
    protected final Hardware hardware;
    protected ElapsedTime targetTime = new ElapsedTime();
    protected ElapsedTime runTime = new ElapsedTime();
    protected boolean finished = false;

    protected Double lastDistanceToTarget = null;

    public MoveToTask(
            @NotNull Scheduler scheduler,
            @NotNull MMoverDataPack mmoverData,
            @NotNull Pose target,
            @Nullable Telemetry telemetry
    ) {
        super(scheduler);
        this.target = target;
        this.tracker = mmoverData.tracking;
        this.speed2Power = mmoverData.speed2Power;
        this.ramps = mmoverData.ramps;
        this.telemetry = telemetry;
        this.loopTimer = mmoverData.loopTimer;
        this.hardware = mmoverData.hardware;
    }

    @Override
    public void invokeOnStart() {
        targetTime.reset();
        runTime.reset();
    }

    @Override
    public void invokeOnTick() {
        Pose current = tracker.getPose();

        double linear = current.linearDistanceTo(target);
        double angular = current.subtractAngle(target);
        if (linear > acceptDist || abs(angular) > RightAuto.ACCEPT_TURN) {
            targetTime.reset();
        }
        // Waits at the target for 0.5 seconds
        if (targetTime.time() > .25) {
            finished = true;
            return;
        }
        // figure out how to get to the target position
        Motion action = tracker.getMotionToTarget(target, hardware);
        double distanceToTarget = sqrt(
                action.forward() * action.forward()
                        + action.right() * action.right()
                        + action.turn() * action.turn());
        double timeNow = runTime.time();
        double rampingSpeed = ramps.ease(
                timeNow,
                distanceToTarget,
                1.0
        );

        double vel;
        if (lastDistanceToTarget == null) vel = 0;
        else vel = (distanceToTarget - lastDistanceToTarget) / loopTimer.getLast();
        lastDistanceToTarget = distanceToTarget;

        double finalSpeed = rampingSpeed + kD * vel;

        action.apply(hardware.driveMotors, Hardware.CALIBRATION, finalSpeed, speed2Power);
        if (telemetry != null) {
            telemetry.addData("forward", action.forward());
            telemetry.addData("right", action.right());
            telemetry.addData("turn (deg)", Math.toDegrees(action.turn()));
        }
        String message = String.format(
                "##%.3f##{\"pose\":[%.6f,%.6f,%.6f],\"motion\":[%.6f,%.6f,%.6f],\"rampingSpeed\":%.6f," +
                        "\"frontLeft\":%.6f,\"frontRight\":%.6f,\"backLeft\":%.6f,\"backRight\":%.6f," +
                        "\"distanceToTarget\":%.6f,\"timer\":%.4f,\"avgTickTime\":%.6f,\"vel\":%.6f," +
                        "\"finalSpeed\":%.6f,\"immediateTickTime\":%.6f}##",
                System.currentTimeMillis() / 1000.0,
                current.x(), current.y(), current.heading(),
                action.forward(), action.right(), action.turn(),
                rampingSpeed,
                action.getLastFL(), action.getLastFR(), action.getLastBL(), action.getLastBR(),
                distanceToTarget, timeNow,
                loopTimer.getAvg() * 1000,
                vel, finalSpeed,
                loopTimer.getLast() * 1000
        );
        Log.d("DataDump", message);
    }

    @Override
    public boolean invokeIsCompleted() {
        return finished;
    }

    @Override
    public void invokeOnFinish() {
        hardware.driveMotors.setAll(0.0);
    }

    private static final Set<SharedResource> REQUIREMENTS = Set.of(
            Hardware.Locks.DriveMotors
    );

    @Override
    public @NotNull Set<SharedResource> requirements() {
        return REQUIREMENTS;
    }
}
