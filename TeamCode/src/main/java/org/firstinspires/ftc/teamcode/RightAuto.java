package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.BlinkLightsTask;
import org.firstinspires.ftc.teamcode.hardware.HClawProxy;
import org.firstinspires.ftc.teamcode.hardware.HSlideProxy;
import org.firstinspires.ftc.teamcode.hardware.VLiftProxy;
import org.firstinspires.ftc.teamcode.mmooover.EncoderTracking;
import org.firstinspires.ftc.teamcode.mmooover.MMoverDataPack;
import org.firstinspires.ftc.teamcode.mmooover.Pose;
import org.firstinspires.ftc.teamcode.mmooover.Ramps;
import org.firstinspires.ftc.teamcode.mmooover.Speed2Power;
import org.firstinspires.ftc.teamcode.mmooover.tasks.MoveRelTask;
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

@Autonomous // Appear on the autonomous drop down
// STOP POSTING ABOUT DefaultLocale IM TIRED OF SEEING IT
@SuppressLint("DefaultLocale")
public class RightAuto extends LinearOpMode {
    // Constants //
    public static final double ACCEPT_DIST = 1; // inch. euclidean distance
    public static final double ACCEPT_TURN = Math.toRadians(3);
    // power biases
    public static final double CLAW_CLOSE = 0.02;
    private static final RuntimeException NOT_IMPLEMENTED = new RuntimeException("This operation is not implemented");
    final Pose STARTPOS = new Pose(0, 4.20, Math.toRadians(0));
    Hardware hardware;
    private final Runnable setup = () -> {
        hardware.claw.setPosition(CLAW_CLOSE);
        hardware.wrist.setPosition(Hardware.WRIST_UP);
        hardware.arm.setPosition(Hardware.ARM_HALF_SPEC);
//        hardware.arm.setTargetPosition(0);
//        hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        hardware.arm.setPower(0.2);
    };
    EncoderTracking tracker;
    private VLiftProxy vLiftProxy;
    private HSlideProxy hSlideProxy;
    private HClawProxy hClawProxy;
    private Ramps ramps;
    private LoopStopwatch loopTimer;
    private Speed2Power speed2Power;
    private MultitaskScheduler scheduler;

    private MMoverDataPack mmoverData;

    private MoveRelTask moveRel(Pose offset) {
        return new MoveRelTask(
                scheduler, mmoverData, offset, telemetry
        );
    }

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
        return new MoveToTask(
                scheduler, mmoverData, target, telemetry
        );
    }

    private BlinkLightsTask blinkenlights(double seconds) {
        return new BlinkLightsTask(
                scheduler, hardware,
                seconds, true, 4 /* Hz */,
                Hardware.LAMP_RED, Hardware.LAMP_ORANGE, Hardware.LAMP_PURPLE
        );
    }

    private BlinkLightsTask lightColor(double color) {
        return new BlinkLightsTask(
                scheduler, hardware,
                0.0, false, 0,
                0, 0, color
        );
    }

    private void hardwareInit() {
        tracker = new EncoderTracking(hardware, STARTPOS);
        loopTimer = new LoopStopwatch();
        speed2Power = new Speed2Power(0.30); // Set a speed2Power corresponding to a speed of 0.20 seconds
        ramps = new Ramps(
                Ramps.linear(2.0),
                Ramps.linear(1 / 12.0),
//                Easing.power(3.0, 12.0),
                Ramps.LimitMode.SCALE
        );
        //                Easing.power(3.0, 12.0),

        mmoverData = new MMoverDataPack(
                hardware, tracker, loopTimer, speed2Power, ramps
        );

        hardware.sharedHardwareInit();
//        hardware.arm.setPosition(Hardware.ARM_PRE_WALL_PICK);
        hardware.wrist.setPosition(Hardware.WRIST_UP);
    }

    private ITask grab() {
        return groupOf(inner -> inner.add(hClawProxy.aSetClaw(Hardware.FRONT_OPEN))
                .then(hSlideProxy.moveOut())
                .then(hClawProxy.aSetFlip(Hardware.FLIP_DOWN))
                .then(await(200))
                .then(hClawProxy.aSetClaw(Hardware.FRONT_CLOSE_HARD))
                .then(await(250))
                .then(hClawProxy.aSetFlip(Hardware.FLIP_ONE_THIRD))
                .then(hSlideProxy.moveToPreset(HSlideProxy.Position.TRANSFER))
        );
    }

    private ITask drop() {
        return groupOf(
                it -> it.add(vLiftProxy.moveTo(0, 5, 1.0))
                        .then(run(() -> {
                            hClawProxy.setClaw(Hardware.FRONT_CLOSE);
                            hardware.claw.setPosition(Hardware.CLAW_OPEN);
                            hardware.clawTwist.setPosition(Hardware.CLAW_TWIST_INIT);
                            hardware.wrist.setPosition(0);
//                            hardware.arm.setTargetPosition(Hardware.ARM_TRANSFER_POS);
                        }))
//                        .then(await(250))
//                        .then(run(() -> {
//
//                        }))
                        .then(await(400))
                        .then(run(() -> hardware.claw.setPosition(Hardware.CLAW_CLOSE)))
                        .then(await(150))
                        .then(run(() -> {
//                            hardware.arm.setTargetPosition(0);
                            hClawProxy.setClaw(Hardware.FRONT_OPEN);
                        }))
                        .then(await(100))
                        .then(run(() -> hardware.clawFront.setPosition(0.6)))
                        .then(await(250))
                        .then(run(() -> {
                            hardware.wrist.setPosition(Hardware.WRIST_BACK);
                        }))
        );
    }

//    private ITask drop() {
//        return groupOf(it -> it.add(run(() -> hardware.arm.setTargetPosition(Hardware.deg2arm(10))))
//                .then(run(() -> hardware.wrist.setPosition(0.75)))
//                .then(await(200))
//                .then(run(() -> hardware.claw.setPosition(Hardware.CLAW_OPEN)))
//                .then(await(200))
//                .then(run(() -> {
//                    hardware.wrist.setPosition(Hardware.WRIST_BACK);
//                    hardware.arm.setTargetPosition(0);
//                }))
//        );
//    }

    private ITask preScoreSpecimen() {
        return groupOf(it -> it.add(run(() -> {
            hardware.wrist.setPosition(Hardware.WRIST_UP);
            hardware.arm.setPosition(Hardware.ARM_HALF_SPEC);
        })).then(vLiftProxy.target(0)));
    }

    private ITask postScoreSpecimen() {
        return run(() -> {
            hardware.wrist.setPosition(0);
            hardware.arm.setPosition(Hardware.ARM_PRE_WALL_PICK);
        });
    }

    private ITask scoreSpecimen() {
        return groupOf(it -> it.add(run(() -> hardware.arm.setPosition(Hardware.ARM_SPEC)))
                .then(await(300))
                .then(run(() -> hardware.driveMotors.setAll(-0.30)))
                .then(await(500))
                .then(run(() -> hardware.driveMotors.setAll(0)))
                .then(run(() -> hardware.claw.setPosition(Hardware.CLAW_OPEN)))
                .then(await(200))
        );
    }

    private ITask pickSpecimen() {
        return groupOf(it -> it.add(run(() -> {
                            hardware.claw.setPosition(Hardware.CLAW_OPEN);
                            hardware.arm.setPosition(Hardware.ARM_PICKUP_WALL);
                        }))
                        .then(await(100))
                        .then(run(() -> hardware.wrist.setPosition(0)))
                        .then(await(300))
                        .then(run(() -> hardware.claw.setPosition(Hardware.CLAW_CLOSE_HARD)))
                        .then(await(200))
                        .then(vLiftProxy.moveTo(80, 5, 1.0))
        );
    }

    public void runAuto() {
        scheduler.add(new OneShot(scheduler, setup))
                .then(groupOf(a -> {
                    a.add(moveTo(new Pose(32, 20, 0)).asCheckpoint());
                }))
                .then(run(() -> hardware.driveMotors.setAll(0.40)))
                .then(await(300))
                .then(run(() -> hardware.driveMotors.setAll(0)))
                .then(scoreSpecimen())
                .then(groupOf(a -> {
                    a.add(moveTo(new Pose(17.0, -35.5, 0)));
                    a.add(postScoreSpecimen());
                }))
                .then(grab())
                .then(groupOf(a -> {
                    a.add(moveTo(new Pose(11.5, -35.5, Math.toRadians(-150))));
                }))
                .then(run(() -> hardware.clawFront.setPosition(Hardware.FRONT_OPEN)))
                .then(await(300))
                .then(groupOf(a -> {
                    a.add(moveTo(new Pose(4, -27, 0)));
                    a.add(blinkenlights(.5));
                    a.add(hSlideProxy.moveIn());
                }))
                .then(run(() -> hardware.driveMotors.setAll(-0.30)))
                .then(await(300))
                .then(run(() -> hardware.driveMotors.setAll(0)))
                .then(pickSpecimen())
                .then(lightColor(Hardware.LAMP_PURPLE))
                .then(groupOf(a -> {
                    a.add(moveTo(new Pose(22, 14, 0)).asCheckpoint());
                    a.add(await(1000)).then(preScoreSpecimen());
                }))
                .then(moveTo(new Pose(32, 15, 0)).asCheckpoint())
                .then(run(() -> hardware.driveMotors.setAll(0.40)))
                .then(await(300))
                .then(run(() -> hardware.driveMotors.setAll(0)))
                .then(scoreSpecimen())
                .then(groupOf(a -> {
                    a.add(moveTo(new Pose(4, -27, 0)));
                    a.add(blinkenlights(.5));
                    a.add(postScoreSpecimen());
                }))
                .then(run(() -> hardware.driveMotors.setAll(-0.30)))
                .then(await(300))
                .then(run(() -> hardware.driveMotors.setAll(0)))
                .then(pickSpecimen())
                .then(lightColor(Hardware.LAMP_PURPLE))
                .then(groupOf(a -> {
                    a.add(moveTo(new Pose(27, 10, 0)));
                    a.add(await(1000)).then(preScoreSpecimen());
                }))
                .then(run(() -> hardware.driveMotors.setAll(0.4)))
                .then(await(500))
                .then(run(() -> hardware.driveMotors.setAll(0)))
                .then(groupOf(a -> {
                    a.add(scoreSpecimen())
                            .then(postScoreSpecimen())
                            .then(moveTo(new Pose(4, -27, 0)));
                    a.add(hSlideProxy.moveToPreset(HSlideProxy.Position.OUT, 0.0));
                }))
        ;
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
//        AtomicReference<Double> scoredIn = new AtomicReference<>((double) 0);
        double doneIn = 0;

        // queue everything up
        scheduler.add(new BackgroundTasks(
                scheduler, tracker, loopTimer
        ));
//        mainAuto();
        runAuto();

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
//            telemetry.addLine(String.format("scored in %.2fs", scoredIn.get()));
            telemetry.addData("x", tracker.getPose().x()); // Print x attribute for pose
            telemetry.addData("y", tracker.getPose().y()); // Print y attribute for pose
            telemetry.addData("heading (rad)", tracker.getPose().heading()); // Print the heading in radians
            telemetry.addData("heading (deg)", Math.toDegrees(tracker.getPose().heading())); // Print the heading in degrees
            telemetry.addLine(String.format("While running: %.2fms per loop", loopTimer.getAvg() * 1000));
            telemetry.update();
        }
    }

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
}

