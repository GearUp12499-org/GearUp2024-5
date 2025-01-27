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
        double[] config = {1.0, 1.0, 0.0};
        limelight.updatePythonInputs(config);
        limelight.setPollRateHz(100);
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(3);
        limelight.start();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result == null) {
                telemetry.addData("Limelight", "result is null!");
            }

            if (result != null /* && result.isValid() */) {
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Avg Area: ", result.getTa());
                telemetry.addData("Pipeline Number: ", limelight.getStatus().getPipelineIndex());

                double[] pythonOutputs = result.getPythonOutput();
                if (pythonOutputs != null && pythonOutputs.length > 6) {
                    double angle = pythonOutputs[0];
                    telemetry.addData("angle", angle);
                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }

            telemetry.update();
        }
    }
}