package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class FieldCentricBlue
        extends LinearOpMode {
    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        StraferHardware hardware=new StraferHardware(hardwareMap);
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "gyro");
        gyro = (IntegratingGyroscope)navxMicro;

       // Servo hand=hardwareMap.get(Servo.class,"hand");
        //DcMotor backRight=hardwareMap.get(DcMotor.class,"backRight");
        //DcMotor backLeft=hardwareMap.get(DcMotor.class,"backLeft");
        //DcMotor frontRight=hardwareMap.get(DcMotor.class,"frontRight");
        //DcMotor frontLeft=hardwareMap.get(DcMotor.class,"frontLeft");
       //  backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        // backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        DistanceSensor sensor=hardwareMap.get(DistanceSensor.class,"distance");
        //      ColorSensor ribbit = hardwareMap.get(ColorSensor.class, "colorSensor");

        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        // Wait until the gyro calibration is complete
        timer.reset();
        while (navxMicro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            Thread.sleep(50);
        }
        telemetry.log().clear();
        telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear();
        telemetry.update();

        waitForStart();
        telemetry.log().clear();




        if (isStopRequested()) return;
        double distance = 100;
        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double botheading = angles.firstAngle;
            double servo = gamepad1.right_trigger;
            double servoclose = gamepad1.left_trigger;

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
            telemetry.addData("distance", distance);
            //  telemetry.addData("color",color);

            AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
            // Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            telemetry.addLine()
                    .addData("dx", formatRate(rates.xRotationRate))
                    .addData("dy", formatRate(rates.yRotationRate))
                    .addData("dz", "%s deg/s", formatRate(rates.zRotationRate));

            telemetry.addLine()
                    .addData("heading", formatAngle(angles.angleUnit, angles.firstAngle))
                    .addData("roll", formatAngle(angles.angleUnit, angles.secondAngle))
                    .addData("pitch", "%s deg", formatAngle(angles.angleUnit, angles.thirdAngle));
            telemetry.update();
            turn(90);
            idle();

        }

    }

    String formatRate(float rate) {
        return String.format("%.3f", rate);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format("%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    /////////////////////////////////////////////////////////////////////////////////////////////
    public void turn(double target) {
        StraferHardware hardware=new StraferHardware(hardwareMap);
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
        }
            else{
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
    }


