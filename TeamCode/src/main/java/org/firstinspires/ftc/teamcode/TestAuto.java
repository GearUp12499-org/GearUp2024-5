package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous
public class TestAuto

        extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor backRight=hardwareMap.get(DcMotor.class,"backRight");
        DcMotor backLeft=hardwareMap.get(DcMotor.class,"backLeft");
        DcMotor frontRight=hardwareMap.get(DcMotor.class,"frontRight");
        DcMotor frontLeft=hardwareMap.get(DcMotor.class,"frontLeft");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DistanceSensor sensor=hardwareMap.get(DistanceSensor.class,"distance");

        waitForStart();
        backRight.setPower(0.3);
        frontRight.setPower(0.3);
        backLeft.setPower(0.3);
        frontLeft.setPower(0.3);
        while (opModeIsActive()){
            double distance = sensor.getDistance(INCH);
            telemetry.addData("distance", distance);
            telemetry.update();
            if (distance < 7){
                backRight.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                frontLeft.setPower(0);
            }
        }

    }
}
//end class.
