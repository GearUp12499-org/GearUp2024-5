package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.HClawProxy;
import org.firstinspires.ftc.teamcode.hardware.HSlideProxy;
import org.firstinspires.ftc.teamcode.hardware.VLiftProxy;
import org.firstinspires.ftc.teamcode.Hardware.Locks;

import java.util.function.Consumer;

import dev.aether.collaborative_multitasking.ITask;
import dev.aether.collaborative_multitasking.MultitaskScheduler;
import dev.aether.collaborative_multitasking.OneShot;
import dev.aether.collaborative_multitasking.Scheduler;
import dev.aether.collaborative_multitasking.SharedResource;
import dev.aether.collaborative_multitasking.TaskGroup;
import dev.aether.collaborative_multitasking.ext.Pause;
import dev.aether.collaborative_multitasking.ITaskWithResult;

@TeleOp
public class FixStuffTeleOp extends LinearOpMode {

    private Hardware hardware;
    private MultitaskScheduler scheduler;
    private VLiftProxy vLiftProxy;
    private HSlideProxy hSlideProxy;
    private HClawProxy hClawProxy;

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

    private void abandonLock(SharedResource theLockInQuestion) {
        scheduler.filteredStop(it -> it.requirements().contains(theLockInQuestion), true, true);
    }

    private void hardwareInit() {

//        hardware.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        hardware.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        hardware.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        hardware.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        hardware.rightFlip.setPosition(Hardware.FLIP_UP);
//        hardware.clawFront.setPosition(Hardware.FRONT_CLOSE);
//
//        hardware.arm.setTargetPosition(0);
//        hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        hardware.arm.setPower(0.3);
//       // hardware.wrist.setPosition(Hardware.WRIST_BACK);
       hardware.claw.setPosition(Hardware.CLAW_CLOSE);
        hardware.clawTwist.setPosition(Hardware.CLAW_TWIST_INIT);
        // we don't have the proxy object to handle this for us
        // so manually implement the inversion
        hardware.horizontalRight.setPosition(Hardware.RIGHT_SLIDE_IN);
        hardware.horizontalLeft.setPosition(1.05 - Hardware.RIGHT_SLIDE_IN);

        hardware.colorLeft.setPosition(Hardware.LAMP_PURPLE);
        hardware.colorRight.setPosition(Hardware.LAMP_PURPLE);

    }

    public void runOpMode() {
        scheduler = new MultitaskScheduler();
        hardware = new Hardware(hardwareMap);
        hardwareInit();

//        vLiftProxy = scheduler.add(new VLiftProxy(scheduler, hardware.verticalLift));
        hSlideProxy = scheduler.add(new HSlideProxy(scheduler, hardware));
        hClawProxy = scheduler.add(new HClawProxy(scheduler, hardware));

        scheduler = new MultitaskScheduler();
        vLiftProxy = scheduler.add(new VLiftProxy(scheduler, hardware.verticalLift));
        hSlideProxy = scheduler.add(new HSlideProxy(scheduler, hardware));
        hClawProxy = scheduler.add(new HClawProxy(scheduler, hardware));

        waitForStart();
        if (isStopRequested()) return;

        boolean wasX = false;

        while (opModeIsActive()) {
            if (gamepad1.b) {
                SlideIn();
            }
            boolean isX = gamepad1.x;
//            if (isX && !wasX) {
//                scheduler.add(GetSpec());
//            }
            wasX = isX;


            if (gamepad1.right_bumper){
                FourthSample();
            }

            if (gamepad1.left_bumper){
                FourthSample2();
            }
            if (gamepad2.dpad_down){
                flipDown();
            }
            if (gamepad2.dpad_up){
                flipUp();
            }
            if (gamepad2.right_bumper){
                close();
            }
            if(gamepad2.a){
                transfer();
            }
            if (gamepad2.b){
                SlideOut();
            }
            if (gamepad2.x){
                pickOffWall();
            }
            if (gamepad2.y){
                scoreSpecimen();
            }
            if (gamepad2.right_trigger>0.5){
                scoreHigh1();
            }
            if (gamepad2.left_trigger > 0.5){
                scoreHigh2();
            }
            telemetry.addData("slidePos", hardware.horizontalLeft.getPosition());
            telemetry.addData("slidePos2", hardware.horizontalRight.getPosition());
            telemetry.addData("leftFlip",hardware.leftFlip.getPosition());
            telemetry.addData("rightFlip",hardware.rightFlip.getPosition());

            telemetry.update();
            scheduler.tick();
        }
    }

//    public ITask GetSpec() {
//        return groupOf(it -> it.add(run(() -> hardware.claw.setPosition(Hardware.CLAW_OPEN)))
//                        .then(await(200))
//                        .then(run(() -> hardware.wrist.setPosition(Hardware.WRIST_UP)))
//                        .then(await(200))
//                        .then(run(() -> hardware.arm.setTargetPosition(65)))
//                        .then(await(500))
//                        .then(run(() -> hardware.claw.setPosition(Hardware.CLAW_CLOSE)))
//                        .then(await(200))
//                        .then(vLiftProxy.moveTo(50, 3, 0.4))
//                        .then(run(() -> hardware.wrist.setPosition(Hardware.WRIST_BACK)))
////                        .then(await(200))
//                        .then(run(() -> hardware.arm.setTargetPosition(Hardware.deg2arm(10))))
//                        .then(await(200))
//                        .then(vLiftProxy.moveTo(0, 5, 0))
//        );
//    }

    public void SlideOut() {
        hardware.clawFront.setPosition(Hardware.FRONT_OPEN);
        sleep(500);
        hardware.horizontalRight.setPosition(Hardware.RIGHT_SLIDE_OUT);
        hardware.horizontalLeft.setPosition(1.05 - Hardware.RIGHT_SLIDE_OUT);
        sleep(500);
//        hardware.rightFlip.setPosition(Hardware.FLIP_DOWN);
//        hardware.leftFlip.setPosition(1-Hardware.FLIP_DOWN);

    }

    public void SlideIn() {
        hardware.clawFront.setPosition(Hardware.FRONT_CLOSE);
        sleep(500);
        hardware.rightFlip.setPosition(Hardware.FLIP_ONE_THIRD);
        sleep(500);
        hardware.horizontalRight.setPosition(Hardware.SLIDE_OVERSHOOT);
        hardware.horizontalLeft.setPosition(1.05 - Hardware.SLIDE_OVERSHOOT);
        sleep(500);
        hardware.horizontalRight.setPosition(Hardware.RIGHT_SLIDE_IN);
        hardware.horizontalLeft.setPosition(1.05 - Hardware.RIGHT_SLIDE_IN);
        hardware.rightFlip.setPosition(Hardware.FLIP_UP);
    }

    public void FourthSample() {
        hardware.clawFront.setPosition(Hardware.FRONT_OPEN);
        sleep(500);
        double PartialFlip = 0.167;
        hardware.rightFlip.setPosition(PartialFlip);
        sleep(500);
        hardware.clawTwist.setPosition(0.26);
        sleep(500);
        hardware.horizontalRight.setPosition(Hardware.RIGHT_SLIDE_OUT);
        hardware.horizontalLeft.setPosition(1.05 - Hardware.RIGHT_SLIDE_OUT);
        sleep(500);
//        hardware.rightFlip.setPosition(Hardware.FLIP_DOWN);
//        sleep(500);
        hardware.clawFront.setPosition(Hardware.FRONT_CLOSE);
        sleep(500);
        hardware.clawTwist.setPosition(Hardware.CLAW_TWIST_INIT);
        sleep(500);
        hardware.horizontalRight.setPosition(Hardware.SLIDE_OVERSHOOT);
        hardware.horizontalLeft.setPosition(1.05 - Hardware.SLIDE_OVERSHOOT);
        sleep(500);
        hardware.horizontalRight.setPosition(Hardware.RIGHT_SLIDE_IN);
        hardware.horizontalLeft.setPosition(1.05 - Hardware.RIGHT_SLIDE_IN);
        sleep(500);
        hardware.rightFlip.setPosition(Hardware.FLIP_UP);
    }

    public void transfer() {
        hardware.armLeft.setPosition(Hardware.LEFT_ARM_TRANSFER);
        hardware.armRight.setPosition(Hardware.RIGHT_ARM_TRANSFER);
        sleep(700);
        hardware.claw.setPosition(Hardware.CLAW_OPEN);
        hardware.wrist.setPosition(Hardware.WRIST_TRANSFER);
        sleep(500);
        hardware.rightFlip.setPosition(Hardware.FLIP_UP);
        hardware.leftFlip.setPosition(1-Hardware.FLIP_UP);
        sleep(500);
        hardware.horizontalRight.setPosition(Hardware.RIGHT_SLIDE_TRANSFER);
        hardware.horizontalLeft.setPosition(Hardware.LEFT_SLIDE_TRANSFER);
        sleep(600);
        hardware.claw.setPosition(Hardware.CLAW_CLOSE);
        sleep(200);
        hardware.clawFront.setPosition(Hardware.FRONT_OPEN);
        sleep(500);
        hardware.armLeft.setPosition(Hardware.LEFT_ARM_SCORE);
        hardware.armRight.setPosition(Hardware.RIGHT_ARM_SCORE);
    }

    public void FourthSample2() {
        hardware.clawFront.setPosition(Hardware.FRONT_OPEN);
        sleep(500);
//        hardware.rightFlip.setPosition(Hardware.FLIP_DOWN);
//        sleep(500);
        //clawTwist at 90 degrees
        hardware.clawTwist.setPosition(0.8133);
        hardware.horizontalRight.setPosition(Hardware.RIGHT_SLIDE_OUT);
        hardware.horizontalLeft.setPosition(1.05 - Hardware.RIGHT_SLIDE_OUT);
        sleep(600);
        hardware.clawFront.setPosition(Hardware.FRONT_CLOSE);
        sleep(500);
        hardware.clawTwist.setPosition(Hardware.CLAW_TWIST_INIT);
        sleep(500);
        hardware.horizontalRight.setPosition(Hardware.SLIDE_OVERSHOOT);
        hardware.horizontalLeft.setPosition(1.05 - Hardware.SLIDE_OVERSHOOT);
        sleep(500);
        hardware.horizontalRight.setPosition(Hardware.RIGHT_SLIDE_IN);
        hardware.horizontalLeft.setPosition(1.05 - Hardware.RIGHT_SLIDE_IN);
        sleep(500);
        hardware.rightFlip.setPosition(Hardware.FLIP_UP);
    }

    public void DriveToDistance (double target) {
        double error = 100;
        while (error > 0.5){
            double kp = 0.15;
            //double LeftDistance = hardware.distanceFrontLeft.getDistance(DistanceUnit.INCH);
           // double RightDistance = hardware.distanceFrontRight.getDistance(DistanceUnit.INCH);
            //double ADistance = (RightDistance+LeftDistance)/2;
           // error = ADistance - target;
            double power = Math.min(0.3, error * kp);
            hardware.frontRight.setPower(power);
            hardware.frontLeft.setPower(power);
            hardware.backRight.setPower(power);
            hardware.backLeft.setPower(power);
            //telemetry.addData("distance", ADistance);
            telemetry.addData("error", error);
            telemetry.update();
        }
    }

//        public void Vtransfer () {
//            hardware.rightFlip.setPosition(0.32);
//            sleep(500);
//            hardware.arm.setTargetPosition(-195);
//            sleep(500);
//            hardware.claw.setPosition(0.5);
//            sleep(500);
//            hardware.wrist.setPosition(0.6);
//            sleep(500);
//            hardware.rightFlip.setPosition(0.62);
//            sleep(500);
//            hardware.claw.setPosition(0.28);
//
//
//        }


    public void flipDown(){
        hardware.horizontalRight.setPosition(Hardware.RIGHT_SLIDE_OUT);
        hardware.horizontalLeft.setPosition(1.05 - Hardware.RIGHT_SLIDE_OUT);
        sleep(100);
        hardware.rightFlip.setPosition(Hardware.FLIP_DOWN);
        hardware.leftFlip.setPosition(1-Hardware.FLIP_DOWN);
    }
    public void flipUp(){
        hardware.rightFlip.setPosition(Hardware.FLIP_ONE_THIRD);
        hardware.leftFlip.setPosition(1-Hardware.FLIP_ONE_THIRD);
        sleep(350);
        hardware.clawFront.setPosition(Hardware.FRONT_CLOSE-0.07);
        hardware.rightFlip.setPosition(Hardware.FLIP_UP);
        hardware.leftFlip.setPosition(1-Hardware.FLIP_UP);
    }
    public void close(){
        hardware.clawFront.setPosition(Hardware.FRONT_CLOSE);
    }
    public void pickOffWall(){
        abandonLock(vLiftProxy.CONTROL);
        abandonLock(Locks.ArmAssembly);
        abandonLock(Locks.DriveMotors);

        scheduler.add(
                groupOf(it -> it.add(run(() -> hardware.claw.setPosition(Hardware.CLAW_OPEN)))
                        .then(run(() -> hardware.armLeft.setPosition(1)))
                        .then(run(() -> hardware.armRight.setPosition(0)))
                        .then(await(100))
                        .then(run(() -> hardware.wrist.setPosition(0)))
                        .then(await(300))
                        .then(run(() -> hardware.claw.setPosition(Hardware.CLAW_CLOSE)))
                        .then(await(200))
                        .then(vLiftProxy.moveTo(50, 5, 1.0))
                        .then(await(500))
                        .then(vLiftProxy.moveTo(0,5,1))

                ).extraDepends(
                        vLiftProxy.CONTROL,
                        Locks.ArmAssembly
                )
        );
    }

    public void scoreSpecimen() {
        abandonLock(vLiftProxy.CONTROL);
        abandonLock(Locks.ArmAssembly);
        abandonLock(Locks.DriveMotors);
        scheduler.add(
                groupOf(it -> it.add(run(() -> hardware.claw.setPosition(Hardware.CLAW_CLOSE)))
                .then(vLiftProxy.moveTo(Hardware.VLIFT_SCORE_SPECIMEN, 5, 1.0))

        ).extraDepends(
                vLiftProxy.CONTROL,
                Locks.ArmAssembly
                )
        );
    }
    public void scoreHigh1() {
        abandonLock(vLiftProxy.CONTROL);
        abandonLock(Locks.ArmAssembly);
        abandonLock(Locks.DriveMotors);

        scheduler.add(
                groupOf(it -> it.add(run(() -> hardware.claw.setPosition(Hardware.CLAW_CLOSE)))
                        .then(vLiftProxy.moveTo(Hardware.VLIFT_SCORE_HIGH, 5, 2.0))
                        .then(await(500))
                        .then(run(() -> hardware.armLeft.setPosition(Hardware.LEFT_ARM_SCORE)))
                        .then(run(() -> hardware.armRight.setPosition(Hardware.RIGHT_ARM_SCORE)))
                        .then(await(300))
                        .then(run(() -> hardware.wrist.setPosition(1)))
                ).extraDepends(
                        vLiftProxy.CONTROL,
                        Locks.ArmAssembly
                )
        );

    }
    public void scoreHigh2() {
        abandonLock(vLiftProxy.CONTROL);
        abandonLock(Locks.ArmAssembly);
        abandonLock(Locks.DriveMotors);

        scheduler.add(
                groupOf(it -> it.add(run(() -> hardware.claw.setPosition(Hardware.CLAW_OPEN)))
                        .then(await(500))
                        .then(run(() -> hardware.armLeft.setPosition(Hardware.LEFT_ARM_UP)))
                        .then(run(() -> hardware.armRight.setPosition(Hardware.RIGHT_ARM_UP)))
                        .then(run(() -> hardware.wrist.setPosition(Hardware.WRIST_UP)))
                        .then(await(400))
                        .then(vLiftProxy.moveTo(0, 5, 2.0))
                        .then(await(300))
                        .then(run(() -> hardware.armLeft.setPosition(Hardware.LEFT_ARM_TRANSFER)))
                        .then(run(() -> hardware.armRight.setPosition(Hardware.RIGHT_ARM_TRANSFER)))
                        .then(await(300))
                        .then(run(() -> hardware.wrist.setPosition(Hardware.WRIST_TRANSFER)))
                ).extraDepends(
                        vLiftProxy.CONTROL,
                        Locks.ArmAssembly
                )
        );

    }
}

