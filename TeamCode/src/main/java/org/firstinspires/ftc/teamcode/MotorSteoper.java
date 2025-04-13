

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MotorSteoper", group = "Concept")
public class MotorSteoper extends LinearOpMode {

    static final int INCREMENT = 1;     // amount to slew servo each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle

    public static final int MAX = 1425;
    public static final int MIN = -1425;

    int position = 0; // Start at halfway position
    boolean rampUp = true;




    @Override
    public void runOpMode() {
        Hardware hardware = new Hardware(hardwareMap);


        // Connect to servo (Assume Robot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo.");
        telemetry.update();
        waitForStart();
        hardware.horizontal.setTargetPosition(position);
        hardware.horizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.horizontal.setPower(0.4);
//        hardware.horizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Scan servo till stop pressed.
        while (opModeIsActive()) {
            hardware.lightLeft.setPosition(1);


            if (gamepad1.y) {
                // Keep stepping up until we hit the max value.
                position += INCREMENT;
                if (position >= MAX) {
                    position = MAX;
                }
            } else if (gamepad1.a) {
                // Keep stepping down until we hit the min value.
                position -= INCREMENT;
                if (position <= MIN) {
                    position = MIN;
                }
            }

            //0.65 - 0.48 offset
            //0.02 is open and 0.50 is closed.

            // Display the current value
            telemetry.addData("Position", "%5d", position);
            telemetry.addData("Position (read)", "%5d", hardware.horizontal.getCurrentPosition());
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

            hardware.horizontal.setTargetPosition(position);

            // Set the servo to the new position and pause;
            sleep(CYCLE_MS);
            idle();
            //hardware.horizontalLeft.setPosition(1.05-position);

        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.addData("position", position);
        telemetry.update();
      //  hardware.horizontalRight.setPosition(position);
    }
}
