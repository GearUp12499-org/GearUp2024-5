package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mmooover.EncoderTracking;
import org.firstinspires.ftc.teamcode.mmooover.Motion;
import org.firstinspires.ftc.teamcode.mmooover.Pose;
import org.firstinspires.ftc.teamcode.mmooover.Ramps;
import org.firstinspires.ftc.teamcode.mmooover.Speed2Power;
import org.firstinspires.ftc.teamcode.mmooover.tasks.MoveRelTask;
import org.firstinspires.ftc.teamcode.mmooover.tasks.MoveToTask;
import org.firstinspires.ftc.teamcode.utilities.LoopStopwatch;
import org.jetbrains.annotations.NotNull;

import dev.aether.collaborative_multitasking.ITask;
import dev.aether.collaborative_multitasking.MultitaskScheduler;
import dev.aether.collaborative_multitasking.OneShot;
import dev.aether.collaborative_multitasking.Scheduler;
import dev.aether.collaborative_multitasking.TaskTemplate;
import dev.aether.collaborative_multitasking.ext.Pause;
import kotlin.Pair;
import kotlin.Unit;

@Autonomous // Appear on the autonomous drop down
// STOP POSTING ABOUT DefaultLocale IM TIRED OF SEEING IT
@SuppressLint("DefaultLocale")
public class RightAuto extends LinearOpMode {
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

    private static final RuntimeException NOT_IMPLEMENTED = new RuntimeException("This operation is not implemented");

    // Constants //
    public static final double ACCEPT_DIST = 1; // inch. euclidean distance
    public static final double ACCEPT_TURN = Math.toRadians(5);

    // power biases
    public static final Motion.Calibrate CALIBRATION = new Motion.Calibrate(1.0, 1.0, 1.0); // Calibration factors for strafe, forward, and turn.

    Hardware hardware;
    EncoderTracking tracker;
    final Pose PARK = new Pose(4, -36, Math.toRadians(0));
    private Ramps ramps;
    private LoopStopwatch loopTimer;
    private Speed2Power speed2Power;
    private MultitaskScheduler scheduler;

    private MoveToTask moveTo(Pose target) {
        return new MoveToTask(
                scheduler, hardware, target, tracker, loopTimer, speed2Power, ramps, telemetry
        );
    }

    private MoveRelTask moveRel(Pose offset) {
        return new MoveRelTask(
                scheduler, hardware, offset, tracker, loopTimer, speed2Power, ramps, telemetry
        );
    }

    public static final double FLIP_UP = 0.98;
    public static final double FLIP_DOWN = 0.04;
    public static final double H_SLIDE_OUT = 0.35;
    public static final double H_SLIDE_IN = 0.1;
    public static final double CLAW_OPEN = 0.55;
    public static final double CLAW_CLOSE = 0.02;
    public static final double WRIST_UP = 0.46;
    public static final double WRIST_BACK = 0.30;
    public static final double VERTICAL_SLIDE_SPEED = 0.75;

    private ITask run(Runnable it) {
        return new OneShot(scheduler, it);
    }

    private ITask wait(double seconds) {
        return new Pause(scheduler, seconds);
    }

    private Pair<ITask, ITask> specimenWallPick() {
        ITask first = scheduler.add(run(() -> hardware.claw.setPosition(CLAW_OPEN)));
        ITask last = first
                .then(wait(1.000))
                .then(run(() -> hardware.wrist.setPosition(WRIST_UP)))
                .then(wait(1.000))
                .then(run(() -> hardware.arm.setTargetPosition(45)))
                .then(wait(0.500))
                .then(run(() -> hardware.claw.setPosition(CLAW_CLOSE)))
                .then(wait(1.000))
                .then(run(() -> {
                    hardware.verticalSlide.setTargetPosition(300);
                    hardware.verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hardware.verticalSlide.setPower(VERTICAL_SLIDE_SPEED);
                }))
                .then(wait(1.000))
                .then(run(() -> hardware.wrist.setPosition(WRIST_BACK)))
                .then(wait(1.000))
                .then(run(() -> hardware.arm.setTargetPosition(10)))
                .then(wait(1.000))
                .then(run(() -> hardware.verticalSlide.setTargetPosition(0)));
        return new Pair<>(first, last);
    }

    private Pair<ITask, ITask> scoreSpecimen() {
        ITask first = scheduler.add(run(() -> {
            hardware.claw.setPosition(CLAW_CLOSE);
            hardware.verticalSlide.setTargetPosition(710);
            hardware.verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.verticalSlide.setPower(VERTICAL_SLIDE_SPEED);
        }));
        ITask last = first
                .then(wait(1.000))
                .then(run(() -> hardware.arm.setTargetPosition(Hardware.deg2arm(-99))))
                .then(wait(1.000))
                .then(run(() -> hardware.wrist.setPosition(1)))
                .then(wait(1.000))
                .then(moveRel(new Pose(-3.5, 0, 0)))
                .then(run(() -> hardware.claw.setPosition(CLAW_OPEN)))
                .then(wait(0.500))
                // Maybe let the rest of this be async
                .then(run(() -> {
                    hardware.wrist.setPosition(0.28);
                    hardware.arm.setTargetPosition(Hardware.deg2arm(0));
                }))
                .then(wait(1.000))
                .then(run(() -> hardware.verticalSlide.setTargetPosition(0)))
                .then(wait(0.25))
                ;
        return new Pair<>(first, last);
    }

    private final Runnable setup = () -> {
        hardware.claw.setPosition(CLAW_CLOSE);
        hardware.wrist.setPosition(0.28);
        hardware.arm.setTargetPosition(0);
        hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.arm.setPower(0.2);
    };

    public void runAuto() {
        scheduler.add(new OneShot(scheduler, setup))
                .then(moveTo(new Pose(30, 12, 0)))
                .then(scoreSpecimen())
//                .then(wait(2.0))
                .then(moveTo(new Pose(4, -48, 0)));
    }

    @Override
    public void runOpMode() {
        scheduler = new MultitaskScheduler();

        hardware = new Hardware(hardwareMap);
        tracker = new EncoderTracking(hardware);
        loopTimer = new LoopStopwatch();
        speed2Power = new Speed2Power(0.20); // Set a speed2Power corresponding to a speed of 0.15 seconds
        ramps = new Ramps(
                Ramps.linear(2.0),
                Ramps.linear(1 / 12.0),
//                Easing.power(3.0, 12.0),
                Ramps.LimitMode.SCALE
        );
        ElapsedTime finalizeTimer = new ElapsedTime();
//        AtomicReference<Double> scoredIn = new AtomicReference<>((double) 0);
        double doneIn = 0;

        // queue everything up
        scheduler.add(new BackgroundTasks(
                scheduler, tracker, loopTimer
        ));
//        mainAuto();
        runAuto();
        // park
        hardware.claw.setPosition(CLAW_CLOSE);

        telemetry.addLine("Initialized.");
        telemetry.addLine(String.format("%d in queue.", scheduler.taskCount()));
        telemetry.update();

        waitForStart(); // Wait for start button

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
//            telemetry.addLine(String.format("scored in %.2fs", scoredIn.get()));
            telemetry.addData("x", tracker.getPose().x()); // Print x attribute for pose
            telemetry.addData("y", tracker.getPose().y()); // Print y attribute for pose
            telemetry.addData("heading (rad)", tracker.getPose().heading()); // Print the heading in radians
            telemetry.addData("heading (deg)", Math.toDegrees(tracker.getPose().heading())); // Print the heading in degrees
            telemetry.addLine(String.format("While running: %.2fms per loop", loopTimer.getAvg() * 1000));
            telemetry.update();
        }
    }
}

