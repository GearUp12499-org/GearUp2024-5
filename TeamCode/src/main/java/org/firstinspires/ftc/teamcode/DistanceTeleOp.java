package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@TeleOp
public class DistanceTeleOp
extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor backRight=hardwareMap.get(DcMotor.class,"backRight");
        DcMotor backLeft=hardwareMap.get(DcMotor.class,"backLeft");
        DcMotor frontRight=hardwareMap.get(DcMotor.class,"frontRight");
        DcMotor frontLeft=hardwareMap.get(DcMotor.class,"frontLeft");
       backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
       frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        DistanceSensor sensor=hardwareMap.get(DistanceSensor.class,"distance");


        waitForStart();


        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            telemetry.addData("distance", sensor.getDistance(INCH));
            telemetry.update();
        }
    }
}
