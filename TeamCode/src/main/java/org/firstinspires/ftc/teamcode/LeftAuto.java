package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.HClawProxy;
import org.firstinspires.ftc.teamcode.hardware.HSlideProxy;
import org.firstinspires.ftc.teamcode.hardware.VLiftProxy;
import org.firstinspires.ftc.teamcode.limelight.LimelightAuto;
import org.firstinspires.ftc.teamcode.limelight.LimelightDetectionMode;
import org.firstinspires.ftc.teamcode.mmooover.EncoderTracking;
import org.firstinspires.ftc.teamcode.mmooover.MMoverDataPack;
import org.firstinspires.ftc.teamcode.mmooover.Pose;
import org.firstinspires.ftc.teamcode.mmooover.Ramps;
import org.firstinspires.ftc.teamcode.mmooover.Speed2Power;
import org.firstinspires.ftc.teamcode.mmooover.tasks.MoveToTask;
import org.firstinspires.ftc.teamcode.utilities.LoopStopwatch;
import org.jetbrains.annotations.NotNull;

import java.util.function.Consumer;

import dev.aether.collaborative_multitasking.ITask;
import dev.aether.collaborative_multitasking.MultitaskScheduler;
import dev.aether.collaborative_multitasking.OneShot;
import dev.aether.collaborative_multitasking.Scheduler;
import dev.aether.collaborative_multitasking.TaskGroup;
import dev.aether.collaborative_multitasking.TaskTemplate;
import dev.aether.collaborative_multitasking.ext.Pause;
import kotlin.Unit;

// STOP POSTING ABOUT DefaultLocale IM TIRED OF SEEING IT
@SuppressLint("DefaultLocale")
public abstract class LeftAuto extends LinearOpMode {
    private static final RuntimeException NOT_IMPLEMENTED = new RuntimeException("This operation is not implemented");
    final Pose SCORE_HIGH_BASKET = new Pose(5.5 + 2.121320344, 19.5 + 2.121320344, Math.toRadians(-45));
    final Pose FINISH = new Pose(SCORE_HIGH_BASKET.x(), SCORE_HIGH_BASKET.y(), Math.toRadians(0));
    final Pose PARK_BAD_K2 = new Pose(56, -8.5, Math.toRadians(-90));
    final Pose PARK_BAD = new Pose(56, -11.5, Math.toRadians(-90));
    final Pose PARK_BAD_K = new Pose(56, 12, Math.toRadians(-90));
    final Pose PICKUP_4 = new Pose(39, 14, Math.toRadians(90));
    final Pose PARK1 = new Pose(57.5, 0, Math.toRadians(0));
    final Pose PARK2 = new Pose(55.5, -11, Math.toRadians(0));
    final Pose START = new Pose(0, 4.66, Math.toRadians(0));
    Hardware hardware;
    EncoderTracking tracker;
    private VLiftProxy vLiftProxy;
    private HSlideProxy hSlideProxy;
    private HClawProxy hClawProxy;
    private Ramps ramps;
    private LoopStopwatch loopTimer;
    private Speed2Power speed2Power;
    private MultitaskScheduler scheduler;

    private MMoverDataPack mmoverData;

    private ITask run(Runnable it) {
        return new OneShot(scheduler, it);
    }

    private ITask wait(double seconds) {
        return new Pause(scheduler, seconds);
    }

    private ITask await(double milliseconds) {
        return wait(milliseconds / 1000);
    }

    private TaskGroup groupOf(Consumer<Scheduler> contents) {
        return new TaskGroup(scheduler).with(contents);
    }

    private MoveToTask moveTo(Pose target) {
        return moveTo(target, 0.005);
    }

    private MoveToTask moveTo(Pose target, double kd) {
        MoveToTask value = new MoveToTask(
                scheduler, mmoverData, target, telemetry
        );
        value.kD = kd;
        return value;
    }

    private ITask transfer() {
        return groupOf(
                it -> it
                        .add(run(() -> {
                            hardware.claw.setPosition(Hardware.CLAW_OPEN);
                            hardware.wrist.setPosition(Hardware.WRIST_TRANSFER);
                            hardware.flip.setPosition(Hardware.FLIP_UP);
                        }))
                        .then(hSlideProxy.moveToPreset(HSlideProxy.Position.TRANSFER, 0.5))
                        .then(run(() -> {
                            hardware.arm.setPosition(Hardware.ARM_TRANSFER);
                        }))
                        .then(await(300))
                        .then(run(() -> hardware.claw.setPosition(Hardware.CLAW_CLOSE)))
                        .then(await(200))
                        .then(groupOf(inner -> {
                            inner.add(hClawProxy.aSetClaw(Hardware.FRONT_OPEN));
                            inner.add(hSlideProxy.moveToPreset(HSlideProxy.Position.KEEP_CLEAR, 0.2));
                            inner.add(run(() -> hardware.arm.setPosition(Hardware.ARM_UP)));
                        }))
        ).extraDepends(
                hClawProxy.CONTROL_CLAW,
                Hardware.Locks.ArmAssembly
        );
    }

    private ITask pickUpYellow() {
        ITask result = groupOf(inner -> inner
                .add(hClawProxy.aSetFlip(Hardware.FLIP_DOWN_PLUS))
                .then(await(300))
                .then(hClawProxy.aSetClaw(Hardware.FRONT_CLOSE_HARD))
                .then(await(200))
                .then(hClawProxy.aSetFlip(Hardware.FLIP_ONE_THIRD))
                .then(await(100)));
        return result;
    }

    private ITask pickUpYellowPart2() {
        return groupOf(inner -> inner.add(run(() -> {
                    hClawProxy.setClaw(Hardware.FRONT_CLOSE);
                    hardware.clawTwist.setPosition(Hardware.CLAW_TWIST_INIT);
                }))
                .then(groupOf(a -> {
                    a.add(hSlideProxy.moveToPreset(HSlideProxy.Position.TRANSFER, 0.8));
                    a.add(await(250))
                            .then(hClawProxy.aSetClaw(Hardware.FRONT_CLOSE_HARD))
                            .then(hClawProxy.aSetFlip(Hardware.FLIP_UP));
                }))
                .then(transfer()));
    }

    private ITask fourthYellow() {
        return groupOf(inner -> inner.add(run(() -> {
                        hardware.clawTwist.setPosition(Hardware.CLAW_TWIST_90);
                        hClawProxy.setFlip(Hardware.FLIP_DOWN);
                }))
                .then(await(200))
                .then(hClawProxy.aSetFlipClaw(Hardware.FLIP_DOWN_PLUS, Hardware.FRONT_CLOSE_HARD))
                .then(await(200))
                .then(groupOf(inner2 -> {
                    inner2.add(hClawProxy.aSetFlip(Hardware.FLIP_ONE_THIRD));
                    inner2.add(hSlideProxy.moveToPreset(HSlideProxy.Position.HOLD, 0.2));
                }))
                .then(run(() -> hardware.clawTwist.setPosition(Hardware.CLAW_TWIST_INIT)))
        );
    }

    private ITask preScoreHighBasket() {
        return groupOf(inner -> {
            // all of these:
            inner.add(vLiftProxy.moveTo(Hardware.VLIFT_SCORE_HIGH, 10, 1.2));
            inner.add(run(() -> hardware.arm.setPosition(Hardware.ARM_UP)));
        });
    }

    private ITask scoreHighBasket() {
        return groupOf(inner -> inner.add(run(() -> {
                            hardware.arm.setPosition(Hardware.ARM_SCORE_AUTO);
                            hardware.wrist.setPosition(Hardware.WRIST_UP);
                        }))
                        .then(await(300))
                        .then(run(() -> hardware.claw.setPosition(Hardware.CLAW_OPEN)))
                        .then(await(100))
                        .then(run(() -> {
                            hardware.claw.setPosition(Hardware.CLAW_CLOSE_HARD);
                            hardware.wrist.setPosition(Hardware.WRIST_BACK);
                            hardware.arm.setPosition(Hardware.ARM_WAIT_BUT_WORSE);
                        }))
                //.then(run(() -> hardware.arm.setTargetPosition(0)))
        );
    }

    private void hardwareInit() {
        tracker = new EncoderTracking(hardware, START);
        loopTimer = new LoopStopwatch();
        speed2Power = new Speed2Power(0.25); // Set a speed2Power corresponding to a speed of 0.20 seconds
        ramps = new Ramps(
                Ramps.linear(5.0), // t seconds
                Ramps.linear(1 / 5.0), // inches from target
//                Easing.power(3.0, 12.0),
                Ramps.LimitMode.SCALE
        );

        mmoverData = new MMoverDataPack(
                hardware, tracker, loopTimer, speed2Power, ramps
        );

        hardware.sharedHardwareInit();
    }

    private ITask getLimelighted() {
        int myColor = isRed() ? LimelightDetectionMode.RED : LimelightDetectionMode.BLUE;
        myColor |= LimelightDetectionMode.YELLOW;
        return new LimelightAuto(
                scheduler, hardware, mmoverData, hSlideProxy, hClawProxy, myColor
        );
    }

    @Override
    public void runOpMode() {
        scheduler = new MultitaskScheduler();
        hardware = new Hardware(hardwareMap);

        hardwareInit();

        vLiftProxy = scheduler.add(new VLiftProxy(scheduler, hardware.verticalLift));
        hSlideProxy = scheduler.add(new HSlideProxy(scheduler, hardware, HSlideProxy.Position.IN, HSlideProxy.Position.OUT));
        hClawProxy = scheduler.add(new HClawProxy(scheduler, hardware));

        ElapsedTime finalizeTimer = new ElapsedTime();
        double doneIn = 0;

        scheduler.add(new BackgroundTasks(
                scheduler, tracker, loopTimer
        ));
        scheduler
                .add(groupOf(a -> {
                    a.add(moveTo(SCORE_HIGH_BASKET));
                    a.add(preScoreHighBasket());
                }))
                .then(groupOf(a -> {
                    a.add(scoreHighBasket());
                    a.add(hClawProxy.aSetFlipClaw(Hardware.FLIP_ONE_THIRD, Hardware.FRONT_OPEN))
                            .then(hSlideProxy.moveOut());
                }))
                .then(groupOf(a -> {
                    a.add(moveTo(new Pose(18.0, 13.25, Math.toRadians(0))));
                    a.add(vLiftProxy.moveTo(0, 5, 1.0));
                }))
                .then(pickUpYellow())
                .then(groupOf(a -> {
                    a.add(pickUpYellowPart2())
                            .then(preScoreHighBasket());
                    a.add(moveTo(SCORE_HIGH_BASKET));
                }))
                .then(scoreHighBasket())
                .then(groupOf(a -> {
                    a.add(moveTo(new Pose(18.0, 23.50, Math.toRadians(0))));
                    a.add(hClawProxy.aSetFlipClaw(Hardware.FLIP_ONE_THIRD, Hardware.FRONT_OPEN))
                            .then(hSlideProxy.moveOut());
                    a.add(vLiftProxy.moveTo(0, 5, 1.0));
                }))
                .then(pickUpYellow())
                .then(groupOf(a -> {
                    a.add(pickUpYellowPart2())
                            .then(preScoreHighBasket());
                    a.add(moveTo(SCORE_HIGH_BASKET));
                }))
                .then(scoreHighBasket())
                .then(groupOf(a -> {
                    a.add(moveTo(PICKUP_4));
                    a.add(hClawProxy.aSetFlipClaw(Hardware.FLIP_ONE_THIRD, Hardware.FRONT_OPEN))
                            .then(hSlideProxy.moveOut());
                    a.add(vLiftProxy.moveTo(0, 5, 1.0));
                }))
//                // 30.75 + sin(15 deg) * 2.0
//                // 12.75 + cos(15 deg) * 2.0
                .then(fourthYellow())
                .then(groupOf(a -> {
                    a.add(pickUpYellowPart2())
                            .then(preScoreHighBasket());
                    a.add(moveTo(SCORE_HIGH_BASKET));
                }))
                .then(scoreHighBasket())
                .then(groupOf(a -> {
                    // 56, -11.5, -90
                    ITask moveTo = a.add(moveTo(PARK_BAD_K).asCheckpoint());
                    moveTo.then(moveTo(PARK_BAD_K2, 0.01));
                    moveTo.then(hSlideProxy.moveOut());
                    a.add(hClawProxy.aSetFlipClaw(Hardware.FLIP_ONE_THIRD, Hardware.FRONT_OPEN));
                    a.add(vLiftProxy.moveTo(0, 5, 1.0));
                }))
                .then(getLimelighted())
                .then(groupOf(a -> {
                    a.add(transfer())
                            .then(preScoreHighBasket());
                    a.add(moveTo(SCORE_HIGH_BASKET));
                }))
                .then(groupOf(a -> {
                    a.add(scoreHighBasket());
                    a.add(hSlideProxy.moveToPreset(HSlideProxy.Position.OUT, 0.0));
                }))
                .then(groupOf(a -> {
                    a.add(moveTo(FINISH, 0));
                    a.add(vLiftProxy.moveTo(0, 5, 0.1));
                }))
//                .then(run(() -> hardware.driveMotors.setAll(0)));
        ;

        telemetry.addLine("Initialized.");
        telemetry.addLine(String.format("%d in queue.", scheduler.taskCount()));
        telemetry.update();

        while (!isStopRequested() && !opModeIsActive()) {
            hSlideProxy.update();
        }
        telemetry.update();
        finalizeTimer.reset();

        while (scheduler.hasJobs() && opModeIsActive()) {
            scheduler.tick();
            scheduler.displayStatus(true, true,
                    str -> {
                        telemetry.addLine(str);
                        return Unit.INSTANCE;
                    });
            telemetry.update();
        }
        doneIn = finalizeTimer.time();
        while (opModeIsActive()) {
            hardware.driveMotors.setAll(0);
            telemetry.addLine(String.format("done in %.2fs", doneIn));
            telemetry.addData("x", tracker.getPose().x()); // Print x attribute for pose
            telemetry.addData("y", tracker.getPose().y()); // Print y attribute for pose
            telemetry.addData("heading (rad)", tracker.getPose().heading()); // Print the heading in radians
            telemetry.addData("heading (deg)", Math.toDegrees(tracker.getPose().heading())); // Print the heading in degrees
            telemetry.addData("lift power", hardware.verticalLift.getPower()); // Print the heading in radians
            telemetry.addLine(String.format("While running: %.2fms per loop", loopTimer.getAvg() * 1000));
            telemetry.update();
        }
    }

//    private ScoreHighBasket scoreHighBasket() {
//        return new ScoreHighBasket(scheduler, hardware);
//    }

    final static class BackgroundTasks extends TaskTemplate {
        EncoderTracking tracker;
        LoopStopwatch timer;

        public BackgroundTasks(
                @NotNull Scheduler scheduler,
                @NotNull EncoderTracking tracker,
                @NotNull LoopStopwatch timer
        ) {
            super(scheduler);
            this.tracker = tracker;
            this.timer = timer;
        }

        @Override
        public boolean getDaemon() {
            return true;
        }

        @Override
        @NotNull
        public String getName() {
            return "Background Items";
        }

        @Override
        public void invokeOnStart() {
            timer.clear();
        }

        @Override
        public void invokeOnTick() {
            timer.click();
            tracker.step();
        }
    }

    abstract boolean isRed();
}

