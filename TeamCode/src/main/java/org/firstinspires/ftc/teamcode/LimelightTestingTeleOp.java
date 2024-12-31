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
public class LimelightTestingTeleOp extends LinearOpMode {
    private Hardware hardware;
    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        hardware = new Hardware(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(3);
        //Starts polling for data
        limelight.start();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            // List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            /*
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());
            */
            /*
            for(LLResultTypes.FiducialResult fiducial : fiducials){
                int id = fiducial.getFiducialId();
                telemetry.addData("ID: ", id);
            }
            */



            if (result == null) {
                telemetry.addData("Limelight","result is null!");
            }

            if (result != null /* && result.isValid() */) {
                // Pose3D botpose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Avg Area: ", result.getTa());

                List<LLResultTypes.ColorResult> colorTargets = result.getColorResults();
                int counter = 0;
                for (LLResultTypes.ColorResult colorTarget : colorTargets) {
                    ArrayList<Double> corners = new ArrayList<Double>();
                    for(List<Double> eachCorner: colorTarget.getTargetCorners()){
                        if(counter > 1){
                            break;
                        }
                        corners.add(eachCorner.get(counter));
                        counter++;
                        
                    }
                    telemetry.addData("First Value of Corner" + corners.get(0), "SecondValue: " + corners.get(1));
                }


                //telemetry.addData("Avg Dist: ", result.getBotposeAvgDist());
                //telemetry.addData("Botpose", botpose.toString());

            } else {
                telemetry.addData("Limelight", "No data available");
            }

            telemetry.update();

        }
    }
}