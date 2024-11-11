/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * This OpMode scans a single servo back and forward until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "hand" as is found on a Robot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "ServoTeleOp", group = "Concept")
//@Disabled
public class ServoTeleOp extends LinearOpMode {

    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position

    // Define class members
    Servo servo;
    double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;


    @Override
    public void runOpMode() {
        Hardware h = new Hardware(hardwareMap);

        // Connect to servo (Assume Robot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        servo = h.wrist;
        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo.");
        telemetry.update();
        waitForStart();

        double wrist = 0.5;
        double twist = 0.5;
        double claw = 0.5;
        int arm = 0;
        h.arm.setTargetPosition(arm);
        h.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        h.arm.setPower(0.7);

        // Scan servo till stop pressed.
        while (opModeIsActive()) {

            // slew the servo, according to the rampUp (direction) variable.
            if (gamepad1.y) {
                // Keep stepping up until we hit the max value.
                claw += INCREMENT;
                if (claw >= 1.0) {
                    claw = 1.0;
                }
            } else if (gamepad1.a) {
                // Keep stepping down until we hit the min value.
                claw -= INCREMENT;
                if (claw <= 0) {
                    claw = 0;
                }
            }
            if (gamepad1.dpad_up) {
                // Keep stepping up until we hit the max value.
                wrist += INCREMENT;
                if (wrist >= 1.0) {
                    wrist = 1.0;
                }
            } else if (gamepad1.dpad_down) {
                // Keep stepping down until we hit the min value.
                wrist -= INCREMENT;
                if (wrist <= 0) {
                    wrist = 0;
                }
            }
            if (gamepad1.x) {
                // Keep stepping up until we hit the max value.
                twist += INCREMENT;
                if (twist >= 1.0) {
                    twist = 1.0;
                }
            } else if (gamepad1.a) {
                // Keep stepping down until we hit the min value.
                twist -= INCREMENT;
                if (twist <= 0) {
                    twist = 0;
                }
            }
            double stick = -gamepad1.left_stick_y;
            if (stick > 0.7) {
                arm += 5;
            } else if (stick < -0.7) {
                arm -= 5;
            }
            h.wrist.setPosition(wrist);
            h.twist.setPosition(twist);
            h.claw.setPosition(claw);
            h.arm.setTargetPosition(arm);
            //0.02 is open and 0.50 is closed.

            // Display the current value
            telemetry.addData("wrist", "%5.2f", wrist);
            telemetry.addData("twist", "%5.2f", twist);
            telemetry.addData("claw", "%5.2f", claw);
            telemetry.addData("arm", "%5.2f", arm);
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

            // Set the servo to the new position and pause;
//            servoMoves();
//            wrist();
            sleep(CYCLE_MS);
            idle();
//aa
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }

    public void servoMoves() {
        servo = hardwareMap.get(Servo.class, "testServo");
        final double open = 0.02;
        final double close = 0.55;
        if (gamepad2.left_bumper) {
            servo.setPosition(0.02);
        } else if (gamepad2.right_bumper) {
            servo.setPosition(0.55);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    public void wrist() {
        servo = hardwareMap.get(Servo.class,"twist");
        StraferHardware hardware = new StraferHardware(hardwareMap);
         final double MAX_WRIST_POS = 1;     // Maximum rotational position
         final double MIN_WRIST_POS = 0.0;     // Minimum rotational position
        if(gamepad2.right_stick_x>=0.5 && gamepad2.right_stick_y>=-0.25 && gamepad2.right_stick_y<=0.25){
            hardware.twist.setPosition(MAX_WRIST_POS);
        } else if (gamepad2.right_stick_x<=-0.5 && gamepad2.right_stick_y>=-0.5 && gamepad2.right_stick_y<=0.5){
            hardware.twist.setPosition(MIN_WRIST_POS);
        }
    }
}

      
//end class.
