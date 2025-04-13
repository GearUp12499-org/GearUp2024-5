package org.firstinspires.ftc.teamcode;

import static androidx.core.math.MathUtils.clamp;
import static java.lang.Math.abs;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.Locks;
import org.firstinspires.ftc.teamcode.hardware.HClawProxy;
import org.firstinspires.ftc.teamcode.hardware.HSlideProxy;
import org.firstinspires.ftc.teamcode.hardware.VLiftProxy;
import org.firstinspires.ftc.teamcode.limelight.LimelightDetectionMode;
import org.firstinspires.ftc.teamcode.limelight.LimelightSearch;
import org.firstinspires.ftc.teamcode.mmooover.EncoderTracking;
import org.firstinspires.ftc.teamcode.mmooover.MMoverDataPack;
import org.firstinspires.ftc.teamcode.mmooover.Ramps;
import org.firstinspires.ftc.teamcode.mmooover.Speed2Power;
import org.firstinspires.ftc.teamcode.utilities.LoopStopwatch;

import java.util.function.Consumer;

import dev.aether.collaborative_multitasking.ITask;
import dev.aether.collaborative_multitasking.ITaskWithResult;
import dev.aether.collaborative_multitasking.MultitaskScheduler;
import dev.aether.collaborative_multitasking.OneShot;
import dev.aether.collaborative_multitasking.Scheduler;
import dev.aether.collaborative_multitasking.SharedResource;
import dev.aether.collaborative_multitasking.TaskGroup;
import dev.aether.collaborative_multitasking.ext.Pause;
import dev.aether.collaborative_multitasking.ext.While;
import kotlin.Unit;
import kotlin.jvm.functions.Function0;


public abstract class MecanumTeleOp2 extends LinearOpMode {
    /**
     * How much you need to push the joysticks to stop autonomous code.
     */
    public static final double PUSH_TO_BYPASS = 0.20;
    double wristPos = 0.28;
    int highBasketTicks = 2180;
    LimelightSearch activeSearchTask = null;
    private Hardware hardware;
    private MultitaskScheduler scheduler;
    private NavxMicroNavigationSensor navxMicro;
    private HSlideProxy hSlideProxy;
    private HClawProxy hClawProxy;
    private EncoderTracking tracker;
    private Ramps ramps;
    private LoopStopwatch loopTimer;
    private Speed2Power speed2Power;
    private org.firstinspires.ftc.teamcode.hardware.VLiftProxy vLiftProxy;
    private double heading = 0.0;
    private MMoverDataPack mmoverData;
    private boolean enableYellow = true;

    /**
     * Forces any tasks using this lock to be stopped immediately.
     *
     * @param theLockInQuestion which lock to forcibly acquire
     */
    private void abandonLock(SharedResource theLockInQuestion) {
        scheduler.filteredStop(it -> it.requirements().contains(theLockInQuestion), true, true);
    }

    private void cancelInQueue(SharedResource theLock) {
        scheduler.filteredStop(it -> it.requirements().contains(theLock) && it.getState() == ITask.State.NotStarted, true, true);
    }

    private OneShot run(Runnable target) {
        return new OneShot(scheduler, target);
    }

    private Pause await(double milliseconds) {
        return new Pause(scheduler, milliseconds / 1000.0);
    }

    private While doWhile(Function0<Boolean> condition, Runnable action) {
        return new While(scheduler, condition, action);
    }

    private TaskGroup groupOf(Consumer<Scheduler> contents) {
        return new TaskGroup(scheduler).with(contents);
    }

    private void hardwareInit() {
        tracker = new EncoderTracking(hardware);
        tracker.setOrientationProvider(() -> heading);
        loopTimer = new LoopStopwatch();
        speed2Power = new Speed2Power(0.25); // Set a speed2Power corresponding to a speed of 0.20 seconds
        ramps = new Ramps(
                Ramps.linear(2.0),
                Ramps.linear(1 / 12.0),
//                Easing.power(3.0, 12.0),
                Ramps.LimitMode.SCALE
        );

        mmoverData = new MMoverDataPack(
                hardware, tracker, loopTimer, speed2Power, ramps
        );

        hardware.sharedHardwareInit();
        hardware.arm.setPosition(Hardware.ARM_WAIT);
        hardware.claw.setPosition(Hardware.CLAW_OPEN);
        hardware.limelight.setPollRateHz(100);
        hardware.limelight.pipelineSwitch(6);

        navxMicro = hardware.gyro;
    }

    /// //////////////////////////////////////////

    @Override
    public void runOpMode() {
        // this scheduler will only be used in init
        scheduler = new MultitaskScheduler();

        hardware = new Hardware(hardwareMap);
        hardwareInit();

        // this scheduler will be used for the restf
        MultitaskScheduler mainScheduler = new MultitaskScheduler();
        vLiftProxy = mainScheduler.add(new VLiftProxy(mainScheduler, hardware.verticalLift));
        hSlideProxy = mainScheduler.add(new HSlideProxy(mainScheduler, hardware, HSlideProxy.Position.TRANSFER, HSlideProxy.Position.OUT));
        hClawProxy = mainScheduler.add(new HClawProxy(mainScheduler, hardware));

        ElapsedTime timer = new ElapsedTime();

        telemetry.log().add("Calibrating gyroscope. Don't touch...");

        scheduler.add(doWhile(
                navxMicro::isCalibrating,
                () -> {
                    telemetry.addData("Calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
                    telemetry.update();
                }
        ));

        // Wait until the gyro calibration is complete
        timer.reset();

        scheduler.runToCompletion(() -> !isStopRequested());

        // this is the scheduler that will be used during the main program
        scheduler = mainScheduler;

        telemetry.log().clear();
        telemetry.log().add("Set and ready to roll!");
        telemetry.clear();

        waitForStart();
        if (isStopRequested()) return;

        boolean isFlipIn = false;
        boolean isFlipOut = false;
        boolean isPreparePick = false;
        boolean isSpecimenPick = false;
        boolean isScoreHigh = false;
        boolean isScoreHigh2 = false;
        boolean isScoreSpecimen = false;
        boolean isScoreSpecimen2 = false;
        boolean isTx = false;
        boolean isSwapYellow = false;
        boolean isSetUpAsc = false;
        boolean isAutodetect = false;

        double yaw_offset = 0.0;
        while (opModeIsActive()) {
            telemetry.addData("Detect yellows? ", enableYellow ? "YES!!" : "NO!!");
            telemetry.addLine("press gp2 right dpad to swap");

            Orientation angles = navxMicro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            if (gamepad1.back) {
                yaw_offset = angles.firstAngle;
            }
            double botheading = angles.firstAngle - yaw_offset;
            telemetry.addData("Heading", formatAngle(angles.angleUnit, botheading));
            heading = botheading;
//                    .addData("heading", formatAngle(angles.angleUnit, angles.firstAngle))
//                    .addData("roll", formatAngle(angles.angleUnit, angles.secondAngle))
//                    .addData("pitch", "%s deg", formatAngle(angles.angleUnit, angles.thirdAngle))

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (abs(y) + abs(x) >= PUSH_TO_BYPASS) abandonLock(Locks.DriveMotors);

            if (!scheduler.isResourceInUse(Locks.DriveMotors)) {
                double rotX = x * Math.cos(-botheading) - y * Math.sin(-botheading);
                double rotY = x * Math.sin(-botheading) + y * Math.cos(-botheading);
                rotX *= 1.1; // Counteract imperfect strafing

                telemetry.addLine()
                        .addData("rotX", rotX)
                        .addData("rotY", rotY);
                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(abs(rotY) + abs(rotX) + abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                double maxSpeed = 1.0;
                double slowSpeed = 0.5;
                double currentSpeed = maxSpeed;
                if (gamepad1.left_bumper) {
                    currentSpeed = slowSpeed;
                }
                hardware.frontLeft.setPower(frontLeftPower * currentSpeed);
                hardware.backLeft.setPower(backLeftPower * currentSpeed);
                hardware.frontRight.setPower(frontRightPower * currentSpeed);
                hardware.backRight.setPower(backRightPower * currentSpeed);

                telemetry.addData("fl power", frontLeftPower);
                telemetry.addData("fr power", frontRightPower);
                telemetry.addData("bl power", backLeftPower);
                telemetry.addData("br power", backRightPower);
            }
            lamps();
            wrist();
            claw();
            stepper();
            //arm();

            boolean shouldScoreHigh = gamepad2.left_trigger > 0.5;
            boolean shouldScoreHigh2 = gamepad2.right_trigger > 0.5;
            if (shouldScoreHigh && !isScoreHigh) {
                ScoreHighBasket1();
            }
            if (shouldScoreHigh2 && !isScoreHigh2) {
                ScoreHighBasket2();
            }
            boolean shouldPreparePick = gamepad2.a;
            if (shouldPreparePick && !isPreparePick) {
                prePickWall();
            }

            boolean shouldSpecimenPick = gamepad2.b;
            if (shouldSpecimenPick && !isSpecimenPick) {
                specimenWallPick();
            }
            boolean shouldScoreSpecimen = gamepad2.dpad_left;
            if (shouldScoreSpecimen && !isScoreSpecimen) {
                spec1();
            }
            boolean shouldScoreSpecimen2 = gamepad2.back;
            if (shouldScoreSpecimen2 && !isScoreSpecimen2) {
                spec2();
            }
            boolean shouldTx = gamepad2.x;
            if (shouldTx && !isTx) {
                transfer();
            }

            boolean shouldSetUpAsc = gamepad1.b;
            if (shouldSetUpAsc && !isSetUpAsc) {
                scheduler.add(groupOf(a -> {
                    a.add(hSlideProxy.moveToPreset(HSlideProxy.Position.OUT));
                    a.add(run(() -> hardware.arm.setPosition(Hardware.ARM_PRE_WALL_PICK)));
                }));
            }

            double ascPow = 0.0;
            if (gamepad1.dpad_up) {
                ascPow = 1.0;
            }
            if (gamepad1.dpad_down) {
                ascPow = -1.0;
            }
            hardware.leftAscent.setPower(ascPow);
            hardware.rightAscent.setPower(ascPow);

            boolean twist90 = gamepad1.y;
            boolean untwist90 = gamepad1.a;
            if (hSlideProxy.isOut() && !scheduler.isResourceInUse(hSlideProxy.CONTROL) && !scheduler.isResourceInUse(hClawProxy.CONTROL_FLIP)) {
                if (twist90) hardware.clawTwist.setPosition(0.83);
                if (untwist90) hardware.clawTwist.setPosition(Hardware.CLAW_TWIST_INIT);
            }

            boolean shouldSwapYellow = gamepad2.dpad_right;
            if (shouldSwapYellow && !isSwapYellow) enableYellow = !enableYellow;

            boolean shouldFlipIn = gamepad1.right_trigger > 0.5;
            if (shouldFlipIn && !isFlipIn) Flipin();
            boolean shouldFlipOut = gamepad1.left_trigger > 0.5;
            if (shouldFlipOut && !isFlipOut) Flipout();


            boolean shouldAutodetect = gamepad2.y;
            if (shouldAutodetect && !isAutodetect) {
                autodetect();
            }

            isSpecimenPick = shouldSpecimenPick;
            isPreparePick = shouldPreparePick;
            isFlipIn = shouldFlipIn;
            isFlipOut = shouldFlipOut;
            isScoreHigh = shouldScoreHigh;
            isScoreHigh2 = shouldScoreHigh2;
            isScoreSpecimen = shouldScoreSpecimen;
            isScoreSpecimen2 = shouldScoreSpecimen2;
            isTx = shouldTx;
            isSwapYellow = shouldSwapYellow;
            isSetUpAsc = shouldSetUpAsc;
            isAutodetect = shouldAutodetect;

            int verticalPosition = hardware.verticalLift.getCurrentPosition();

            scheduler.tick();
            tracker.step();
            loopTimer.click();
            telemetry.addData("Wrist Position", hardware.wrist.getPosition());
            telemetry.addData("Claw Position", hardware.claw.getPosition());
            telemetry.addData("Vertical position", verticalPosition);
            telemetry.addData("Est pose", tracker.getPose());
//            telemetry.addData("ascLeft calib", hardware.ascent.getLeftPosition());
//            telemetry.addData("ascRight calib", hardware.ascent.getRightPosition());
//            telemetry.addData("asc target", hardware.ascent.getTargetPosition());
            scheduler.displayStatus(false, true, (str) -> {
                telemetry.addLine(str);
                return Unit.INSTANCE;
            });

            telemetry.update();
        }

    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    // lifts the vertical slides to a target position in ticks

    @SuppressLint("DefaultLocale")
    String formatDegrees(double degrees) {
        return String.format("%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    private ITaskWithResult<Boolean> targetLift(int targetPosition) {
        abandonLock(vLiftProxy.CONTROL);
        return scheduler.add(vLiftProxy.moveTo(targetPosition, 5, 3.0));
    }

    private void lamps() {
        if (scheduler.isResourceInUse(Locks.Signals)) return;

//        @ColorInt int color = hardware.clawColor.argb();
        // extract the components from the packed number
//        int red = (color >> 16) & 0xff;
//        int green = (color >> 8) & 0xff;
//        int blue = color & 0xff;
        int red = hardware.clawColor.red();
        int green = hardware.clawColor.green();
        int blue = hardware.clawColor.blue();

        if (blue - green > 100 && blue - red > 100) {
            hardware.colorRight.setPosition(Hardware.LAMP_BLUE);
            hardware.colorLeft.setPosition(Hardware.LAMP_BLUE);
//            telemetry.addLine("blue");
        } else if (red - blue > 100 && red - green > 100) {
//            telemetry.addLine("red");
            hardware.colorRight.setPosition(Hardware.LAMP_RED);
            hardware.colorLeft.setPosition(Hardware.LAMP_RED);
        } else if (green - blue > 100 && green - red > 100 && red >= 350) {
//            telemetry.addLine("yellow");
            hardware.colorRight.setPosition(Hardware.LAMP_YELLOW);
            hardware.colorLeft.setPosition(Hardware.LAMP_YELLOW);
        } else {
            hardware.colorRight.setPosition(0);
            hardware.colorLeft.setPosition(0);
        }
    }

//    private double getArmPosDeg() {
//        double rotations = hardware.arm.getCurrentPosition() / Hardware.spinTickPerRev;
//        // 0 = straight down
//        return rotations * 360.0;
//    }

//    private void arm() {
//        // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
//        // 537.7 ppr
//        DcMotor arm = hardware.arm;
//        double stick_pos = -gamepad2.right_stick_y;
//        double rotations = arm.getCurrentPosition() / Hardware.spinTickPerRev;
//        double degrees = rotations * 360.0; // 0 = straight down
//        double tRotations = arm.getTargetPosition() / Hardware.spinTickPerRev;
//        double tDegrees = tRotations * 360.0; // 0 = straight down
//        // Negative: towards front;
//        // Positive: towards back.
//        // Exclusion zone 0 to -25deg whe lift < 6in.
//        // [removed if statement, disabled]
//        // Full* clearance
//        boolean shouldWrite = true;
//        if (stick_pos > 0.7 && tDegrees <= 110) {
//            tDegrees += 1;
//        } else if (stick_pos < -0.7 && tDegrees >= -110) {
//            tDegrees -= 1;
//        } else {
//            shouldWrite = false;
//        }
//        if (shouldWrite) {
//            abandonLock(Locks.ArmAssembly);
//            arm.setTargetPosition(Hardware.deg2arm(tDegrees));
//        }
//        telemetry.addData("arm deg", degrees);
//        telemetry.addData("arm target deg", tDegrees);
//    }

    public void wrist() {
        boolean write = false;
        if (gamepad2.left_stick_y >= 0.5 && gamepad2.left_stick_x >= -0.25 && gamepad2.left_stick_x <= 0.25) {
            wristPos += 0.01;
            write = true;
        } else if (gamepad2.left_stick_y <= -0.5 && gamepad2.left_stick_x >= -0.25 && gamepad2.left_stick_x <= 0.25) {
            wristPos -= 0.01;
            write = true;
        }
        if (write) {
            abandonLock(Locks.ArmAssembly);
            hardware.wrist.setPosition(wristPos);
        }
        telemetry.addData("Wrist Position", wristPos);
    }

    public void claw() {
        if (gamepad2.left_bumper) {
            abandonLock(Locks.ArmAssembly);
            hardware.claw.setPosition(Hardware.CLAW_OPEN);
        } else if (gamepad2.right_bumper) {
            abandonLock(Locks.ArmAssembly);
            hardware.claw.setPosition(Hardware.CLAW_CLOSE);
        }
    }

    public void resetAll() {
        abandonLock(Locks.ArmAssembly);
        abandonLock(vLiftProxy.CONTROL);
        scheduler.add(groupOf(it -> it.add(run(() -> hardware.claw.setPosition(Hardware.CLAW_OPEN)))
                        .then(await(200))
                        .then(run(() -> {
//                    hardware.arm.setTargetPosition(0);
                            hardware.wrist.setPosition(0.28);
                        }))
                        .then(await(200))
                        .then(vLiftProxy.moveTo(0, 5, 1.0))
        ).extraDepends(Locks.ArmAssembly, vLiftProxy.CONTROL));
    }

    /// ///////////////////////////////////////////////////////////////////////////////////////////////
    public void ScoreHighBasket1() {
        abandonLock(Locks.ArmAssembly);
        abandonLock(vLiftProxy.CONTROL);
        scheduler.add(
                groupOf(inner -> {
                    inner.add(vLiftProxy.moveTo(Hardware.VLIFT_SCORE_HIGH, 5, 2.0));
                    inner.add(run(() -> hardware.claw.setPosition(Hardware.CLAW_CLOSE)));
                    inner.add(run(() -> hardware.arm.setPosition(Hardware.ARM_SCORE)));
                    inner.add(run(() -> hardware.wrist.setPosition(1)));
                }).extraDepends(
                        Locks.ArmAssembly,
                        vLiftProxy.CONTROL
                )
        );
    }

    public void ScoreHighBasket2() {
        // prevent doing this by accident
        if (hardware.verticalLift.getCurrentPosition() < Hardware.VLIFT_SCORE_HIGH - 100) return;
        abandonLock(Locks.ArmAssembly);
        abandonLock(vLiftProxy.CONTROL);
        scheduler.add(groupOf(
                inner -> inner.add(run(() -> hardware.claw.setPosition(Hardware.CLAW_OPEN)))
                        .then(await(300))
                        .then(run(() -> {
                            hardware.arm.setPosition(Hardware.ARM_UP);
                            hardware.wrist.setPosition(Hardware.WRIST_UP);
                        }))
                        .then(await(400))
                        .then(vLiftProxy.moveTo(0, 5, 2.0))
                        .then(await(300))
                        .then(run(() -> {
                            hardware.wrist.setPosition(Hardware.WRIST_TRANSFER);
                            hardware.arm.setPosition(Hardware.ARM_WAIT);
                        }))
        ).extraDepends(
                Locks.ArmAssembly,
                vLiftProxy.CONTROL
        ));
    }

    private void stepper() {
        // this deals with the horizontal slide stuff. we don't have locks (yet) so it's remaining
        // as-is for now
        if (gamepad1.dpad_left) {
            abandonLock(hClawProxy.CONTROL_CLAW);
            hClawProxy.setClaw(Hardware.FRONT_OPEN);
        }
        if (gamepad1.dpad_right) {
            abandonLock(hClawProxy.CONTROL_CLAW);
            hClawProxy.setClaw(Hardware.FRONT_CLOSE_HARD);
        }

        telemetry.addData("FrontClawPos", hClawProxy.getClawPosition());
        telemetry.addData("FlipClawPos", hClawProxy.getFlipPosition());
    }

    public void prePickWall() {
        scheduler.add(groupOf(it -> it.add(run(() -> {
                    hardware.claw.setPosition(Hardware.CLAW_OPEN);
                    hardware.wrist.setPosition(0);
                    hardware.arm.setPosition(Hardware.ARM_PRE_WALL_PICK);
                })).then(vLiftProxy.moveTo(0, 5, 0.5))
        ).extraDepends(Locks.ArmAssembly));
    }

    public void specimenWallPick() {
        abandonLock(Locks.ArmAssembly);
        abandonLock(vLiftProxy.CONTROL);
        scheduler.add(
                groupOf(it -> it.add(run(() -> {
                                    hardware.claw.setPosition(Hardware.CLAW_OPEN);
                                    hardware.arm.setPosition(Hardware.ARM_PICKUP_WALL);
                                }))
                                .then(await(100))
                                .then(run(() -> hardware.wrist.setPosition(0)))
                                .then(await(300))
                                .then(run(() -> hardware.claw.setPosition(Hardware.CLAW_CLOSE_HARD)))
                                .then(await(200))
                                .then(vLiftProxy.moveTo(80, 5, 1.0))
                ).extraDepends(
                        vLiftProxy.CONTROL,
                        Locks.ArmAssembly
                )
        );
    }

    private void spec1() {
        abandonLock(Locks.ArmAssembly);
        scheduler.add(groupOf(it ->
                it.add(run(() -> {
                            hardware.claw.setPosition(Hardware.CLAW_CLOSE_HARD);
                            hardware.arm.setPosition(Hardware.ARM_HALF_SPEC);
                        }))
                        .then(await(200))
                        .then(run(() -> hardware.wrist.setPosition(Hardware.WRIST_UP)))
        ).extraDepends(
                Locks.ArmAssembly
        ));
    }

    private void spec2() {
        scheduler.add(groupOf(it ->
                it.add(run(() -> {
                    hardware.claw.setPosition(Hardware.CLAW_CLOSE_HARD);
                    hardware.arm.setPosition(Hardware.ARM_SPEC);
                }))
        ).extraDepends(
                Locks.ArmAssembly
        ));
    }

    // TODO: Implement at all
//    public void transferAndDrop() {
//        abandonLock(hClawProxy.CONTROL_CLAW);
//        abandonLock(vLiftProxy.CONTROL);
//        abandonLock(Locks.ArmAssembly);
//        transferInternal()
//                .then(groupOf(
//                       it -> it.add(run(() -> hardware.arm.setTargetPosition(Hardware.deg2arm(10))))
//                                .then(await(100))
//                                .then(run(() -> hardware.wrist.setPosition(0.75)))
//                                .then(await(100))
//                                .then(run(() -> hardware.claw.setPosition(Hardware.CLAW_OPEN)))
//                                .then(await(300))
//                                .then(run(() -> {
//                                    hardware.wrist.setPosition(Hardware.WRIST_BACK);
//                                    //hardware.arm.setTargetPosition(0);
//                                }))
//                ).extraDepends(
//                        Locks.ArmAssembly
//                ));
//    }

    public void Flipout() {
        abandonLock(hSlideProxy.CONTROL);
        abandonLock(hClawProxy.CONTROL_CLAW);
        abandonLock(hClawProxy.CONTROL_FLIP);
        scheduler.add(groupOf(
                it -> it.add(hClawProxy.aSetClaw(Hardware.FRONT_OPEN))
                        .then(hSlideProxy.moveOut())
                        .then(hClawProxy.aSetFlip(Hardware.FLIP_DOWN))
                        .then(await(200))
        ).extraDepends(
                hSlideProxy.CONTROL,
                hClawProxy.CONTROL_FLIP,
                hClawProxy.CONTROL_CLAW
        ));
    }

    public void Flipin() {
        abandonLock(hSlideProxy.CONTROL);
        abandonLock(hClawProxy.CONTROL_FLIP);
        scheduler.add(groupOf(
                it -> it.add(hClawProxy.aSetFlip(Hardware.FLIP_ONE_THIRD))
                        .then(run(() -> {
                            hClawProxy.setClaw(Hardware.FRONT_CLOSE);
                            hardware.clawTwist.setPosition(Hardware.CLAW_TWIST_INIT);
                        }))
                        .then(groupOf(a -> {
                            a.add(hSlideProxy.moveTransfer());
                            a.add(await(350))
                                    .then(hClawProxy.aSetClaw(Hardware.FRONT_CLOSE_HARD));
                        }))
                        .then(hClawProxy.aSetFlip(Hardware.FLIP_UP))
        ).extraDepends(
                hSlideProxy.CONTROL,
                hClawProxy.CONTROL_FLIP
        ));
    }

    private SharedResource transferKind = new SharedResource("type: transfer");

    private TaskGroup transferInternal() {
        return scheduler.add(groupOf(
                it -> it
                        .add(run(() -> {
                            hardware.claw.setPosition(Hardware.CLAW_OPEN);
                            hardware.wrist.setPosition(Hardware.WRIST_TRANSFER);
                            hardware.flip.setPosition(Hardware.FLIP_UP);
                        }))
                        .then(hSlideProxy.moveTransfer())
                        .then(run(() -> {
                            hardware.arm.setPosition(Hardware.ARM_TRANSFER);
                        }))
                        .then(await(300))
                        .then(run(() -> hardware.claw.setPosition(Hardware.CLAW_CLOSE)))
                        .then(await(200))
                        .then(hClawProxy.aSetClaw(Hardware.FRONT_OPEN))
                        .then(run(() -> hardware.arm.setPosition(Hardware.ARM_UP)))
                        .then(hSlideProxy.moveToPreset(HSlideProxy.Position.KEEP_CLEAR, 0.2))
        ).extraDepends(
                hClawProxy.CONTROL_CLAW,
                Locks.ArmAssembly,
                transferKind
        ));
    }

    public void transfer() {
//        abandonLock(hClawProxy.CONTROL_CLAW);
//        abandonLock(Locks.ArmAssembly);
        abandonLock(transferKind);
        transferInternal();
    }

    void autodetect() {
        if (activeSearchTask != null && activeSearchTask.getState() == ITask.State.Ticking) {
            activeSearchTask.proceed();
            return;
        }
        abandonLock(Locks.DriveMotors);
        int myColor = isRed() ? LimelightDetectionMode.RED : LimelightDetectionMode.BLUE;
        if (enableYellow) myColor |= LimelightDetectionMode.YELLOW;
        activeSearchTask = scheduler.add(new LimelightSearch(scheduler, hardware, mmoverData, hSlideProxy, hClawProxy, myColor, telemetry));
    }

    // go to MecanumTeleOp2 for functionality
    protected abstract boolean isRed();
}
