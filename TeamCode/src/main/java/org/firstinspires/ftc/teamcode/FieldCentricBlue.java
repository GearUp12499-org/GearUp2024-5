package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import android.media.MediaPlayer;

//import androidx.room.parser.expansion.Position;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
//servo impost did not come with the class. so just incase you make a new one please make sure to put import servo. thank you
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.List;

@TeleOp
public class FieldCentricBlue
        extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            5, 7, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    //this is all copy-pasted from AprilTagYummy
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private CRServo con_servo;
    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        initAprilTag();
        StraferHardware hardware = new StraferHardware(hardwareMap);
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "gyro");
        gyro = (IntegratingGyroscope) navxMicro;


        DistanceSensor sensor = hardwareMap.get(DistanceSensor.class, "distance");
        //      ColorSensor ribbit = hardwareMap.get(ColorSensor.class, "colorSensor");
        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        // Wait until the gyro calibration is complete
        timer.reset();
        while (navxMicro.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            Thread.sleep(50);
        }
        telemetry.log().clear();
        telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear();
        telemetry.update();
        //this makes it so that the servo automatically sets to position when the program is initialised. not started. this is because it comes before the waitForStart. thank you
        hardware.hand.setPosition(1.0);

        waitForStart();
        telemetry.log().clear();


        if (isStopRequested()) return;
        double distance = 100;
        boolean haveTurned = false;
        while (opModeIsActive()) {
            telemetryAprilTag();
            telemetry.update();

            // this is the gamepad controls for the driving. good luck
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            // double MAX_POS = 1.0;
            Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double botheading = angles.firstAngle;


            //Rotate the movement direction counter to the bot's rotation.
            double rotX = x * Math.cos(-botheading) + y * Math.sin(-botheading);
            double rotY = x * Math.sin(-botheading) - y * Math.cos(-botheading);

            rotX = rotX * 1.1;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;


            hardware.frontLeft.setPower(frontLeftPower / 2);
            hardware.backLeft.setPower(backLeftPower / 2);
            hardware.frontRight.setPower(frontRightPower / 2);
            hardware.backRight.setPower(backRightPower / 2);

            // double color= ribbit.argb();
            distance = sensor.getDistance(INCH);
           // telemetry.addData("distance", distance);
            //  telemetry.addData("color",color);

            AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
            // Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

         /*   telemetry.addLine()
                    .addData("dx", formatRate(rates.xRotationRate))
                    .addData("dy", formatRate(rates.yRotationRate))
                    .addData("dz", "%s deg/s", formatRate(rates.zRotationRate));

            telemetry.addLine()
                    .addData("heading", formatAngle(angles.angleUnit, angles.firstAngle))
                    .addData("roll", formatAngle(angles.angleUnit, angles.secondAngle))
                    .addData("pitch", "%s deg", formatAngle(angles.angleUnit, angles.thirdAngle));
            telemetry.update();
*/
            handservo(1.0);

            if (gamepad1.dpad_up) {
                straightline(0.3, 24.0, 0.0);
            }

            if (gamepad1.b) {
                turn2(90);

            }
            if (gamepad1.a) {
                turn2(-90);
            }
            if (gamepad1.dpad_down) {
                crservo(90);
            }
            //Rumble();
            //gamepad controls here.
            idle();

        }

    }
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(aprilTag);
        visionPortal = builder.build();

    }
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y,
                        detection.robotPose.getPosition().z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

    }

    String formatRate(float rate) {
        return String.format("%.3f", rate);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format("%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    /////////////////////////////////////////////////////////////////////////////////////////////

    public void turn(double target) {
        StraferHardware hardware = new StraferHardware(hardwareMap);
        //DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        //DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        //DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        //DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double p = 0.7;
        double power = 0;
        double error = angles.firstAngle - target;
        if (target < 0) {
            while (error > 0) {

                hardware.frontLeft.setPower(power);
                hardware.backLeft.setPower(power);
                hardware.frontRight.setPower(-power);
                hardware.backRight.setPower(-power);

                angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                error = angles.firstAngle - target;
                power = error * (0.013) * p + 0.2;

                telemetry.addData("errorccw", error);
                telemetry.addData("headingccw", angles.firstAngle);
                telemetry.addData("targetccw", target);
                telemetry.update();
            }
        } else {
            while (error < 0) {

                hardware.frontLeft.setPower(-power);
                hardware.backLeft.setPower(-power);
                hardware.frontRight.setPower(power);
                hardware.backRight.setPower(power);

                angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                error = angles.firstAngle - target;
                power = error * (-0.013) * p + 0.2;

                telemetry.addData("errorcw", error);
                telemetry.addData("headingcw", angles.firstAngle);
                telemetry.addData("targetcw", target);
                telemetry.update();

            }
            hardware.frontLeft.setPower(0);
            hardware.backLeft.setPower(0);
            hardware.frontRight.setPower(0);
            hardware.backRight.setPower(0);


        }
    }
    ////////////////////////////////////////////////////////////////////////////////////
    public void crservo(double deltaAngle){
        con_servo=hardwareMap.crservo.get("intakeServo");
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.dpad_left){
                con_servo.setPower(1);
            }
            if (gamepad1.dpad_right){
                con_servo.setPower(-1);
            }
            if (gamepad1.atRest()){
                con_servo.setPower(0);
            }
            telemetry.update();
        }
    }
//////////////////////////////////////////////////////////////////////////////////////////
    public void turn2(double deltaAngle) {
        //turn to an angle accurately.
        ElapsedTime timer = new ElapsedTime();
        StraferHardware hardware = new StraferHardware(hardwareMap);
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double target = angles.firstAngle + deltaAngle;
        double allowedError = 5.0;
        double maxPower = 1;
        double minPower = 0.1;
        double kp = 1.0 / 60.0;
        double timeOutSeconds = 3;

        while (timer.time() < timeOutSeconds) {
            telemetry.addData("timer", timer.time());
            telemetry.update();

            angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double error = target - angles.firstAngle;
            if (error > 180) {
                error -= 360;
            }
            if (error < -180) {
                error += 360;
            }
            if (Math.abs(error) < allowedError) {
                hardware.frontLeft.setPower(0);
                hardware.backLeft.setPower(0);
                hardware.frontRight.setPower(0);
                hardware.backRight.setPower(0);
                return;
            }
            double rampDown = kp * Math.abs(error);
            double power = Math.min(maxPower, rampDown);
            if (power < minPower) {
                power = minPower;
            }
            if (error < 0.0) {
                power = -power;
            }
            hardware.frontLeft.setPower(-power);
            hardware.backLeft.setPower(-power);
            hardware.frontRight.setPower(power);
            hardware.backRight.setPower(power);

        }
    }

    /////////////////////////////////////////////////////////////////////////////////
    public void handservo(double deltaAngle) {
        //hand teleop controls :}
        StraferHardware hardware = new StraferHardware(hardwareMap);

        if (gamepad1.left_bumper) {
            hardware.hand.setPosition(0.0);
        }
        if (gamepad1.right_bumper) {
            hardware.hand.setPosition(1.0);
        }
    }

    //////////////////////////////////////////////////////////////////////////////////
    public void straightline(double power, double distance, double heading) {
        //drives in a straight line from robots current orientation
        ElapsedTime timer = new ElapsedTime();
        StraferHardware hardware = new StraferHardware(hardwareMap);
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double target = angles.firstAngle;
        double kp = 0.9 / 45;
        double p = 0.7;
        double error = angles.firstAngle - target;
        double currentHeading = 0;
        double delta = 0;

        //537.7 is the count per rotation of the gobilda planetary motor.
        //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/?srsltid=AfmBOop1YfJqiOXEErvydFNjwvWoea8tDZOZ1Pz4a9llh_pNC8bSNKKy
        double distancePerTicks = 4 * 3.14 / 537.7;
        int current = hardware.frontLeft.getCurrentPosition();
        while (current < distance / distancePerTicks) {


            angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = angles.firstAngle;
            error = currentHeading - target;
            delta = error * kp;
            current = hardware.frontLeft.getCurrentPosition();

            hardware.frontLeft.setPower(power + delta);
            hardware.backLeft.setPower(power + delta);
            hardware.frontRight.setPower(power - delta);
            hardware.backRight.setPower(power - delta);

            telemetry.addData("timer", timer.time());
            telemetry.addData("headingcw", angles.firstAngle);
            telemetry.addData("error", error);
            telemetry.addData("distance", distancePerTicks);
            telemetry.addData("current position", current);
            telemetry.update();
        }
        hardware.frontRight.setPower(0.0);
        hardware.frontLeft.setPower(0.0);
        hardware.backRight.setPower(0.0);
        hardware.backLeft.setPower(0.0);


    }
////////////////////////////////////////////////////////////////////////////////////////////////
    public void Rumble() {
        StraferHardware hardware = new StraferHardware(hardwareMap);
        ColorSensor ribbit = hardwareMap.get(ColorSensor.class, "colorSensor");
        double red = ribbit.red();
        double blue = ribbit.blue();
        double green = ribbit.green();
        boolean yellow = (green - blue > 100 && green - red > 100 && red >= 350);


        if (yellow) {
            gamepad1.rumble(1000);
        } else {
            gamepad1.rumbleBlips(1);
        }


    }

}




//end class.
