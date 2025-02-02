package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.ArrayList;
import java.util.List;

@TeleOp
public class LimelightTestingTeleOp2 extends LinearOpMode {
    private Hardware hardware;
    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        hardware = new Hardware(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        double[] llrobot = {1.0, 1.0, 0.0}; // {Red, Yellow, Blue}
        limelight.updatePythonInputs(llrobot);
        limelight.setPollRateHz(100);
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(6);
        limelight.start();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                hardware.limelightlight.setPosition(1);
                hardware.clawFront.setPosition(Hardware.FRONT_OPEN);
                boolean foundTarget = false;
                while (!foundTarget) {
                    LLResult result = limelight.getLatestResult();
                    if (gamepad1.b) {
                        foundTarget = true;
                        break;
                    }
                    if (result != null /* && result.isValid() */) {
                        // telemetry.addData("tx", result.getTx());
                        // telemetry.addData("ty", result.getTy());
                        // telemetry.addData("Avg Area: ", result.getTa());
                        // telemetry.addData("Pipeline Number: ", limelight.getStatus().getPipelineIndex());
                        double[] pythonOutputs = result.getPythonOutput();
                        telemetry.addData("==========","==========");
                        for (double output : pythonOutputs) {
                            telemetry.addData("python output", output);
                        }
                        telemetry.update();
                        if (pythonOutputs != null && pythonOutputs.length > 6) {
                            double angle = pythonOutputs[6];
                            double x_coord = pythonOutputs[1];
                            double y_coord = pythonOutputs[2];
                            double w_val = pythonOutputs[3];
                            double h_val = pythonOutputs[4];
                            double ratio = h_val/w_val;
                            telemetry.addData("angle", angle);
                            telemetry.addData("x", x_coord);
                            telemetry.addData("y", y_coord);
                            telemetry.addData("w", w_val);
                            telemetry.addData("h", h_val);
                            telemetry.addData("ratio", ratio);
                            if (ratio > 1.5 && ratio < 2.0) {
                                if (x_coord > 235 && x_coord < 350) {
                                    if (y_coord > 100 && y_coord < 388) {
                                        foundTarget = true;
                                        hardware.clawFlip.setPosition(Hardware.FLIP_DOWN);
                                        sleep(400);
                                        double pickupPosition;
                                        if (angle < 90) {
                                            pickupPosition = Hardware.CLAW_TWIST_INIT - (angle * 0.0037);
                                        } else {
                                            angle = 180 - angle;
                                            pickupPosition = Hardware.CLAW_TWIST_INIT + (angle * 0.0037);
                                        }
                                        double temp = Math.abs(pickupPosition - Hardware.CLAW_TWIST_INIT);
                                        long twist_delay = (long) (100 + (500/0.33)*temp);
                                        telemetry.addData("pickupPosition", pickupPosition);
                                        hardware.clawTwist.setPosition(pickupPosition);
                                        sleep(100);
                                        hardware.clawFront.setPosition(Hardware.FRONT_CLOSE);
                                        sleep(300);
                                        hardware.clawTwist.setPosition(Hardware.CLAW_TWIST_INIT);
                                        sleep(twist_delay);
                                        hardware.clawFlip.setPosition(Hardware.FLIP_UP);
                                    }
                                }
                            }
                        }
                    } else {
                        telemetry.addData("Limelight", "Result is null");
                    }
                }
                hardware.limelightlight.setPosition(0);
            }
            telemetry.update();
        }
    }
}