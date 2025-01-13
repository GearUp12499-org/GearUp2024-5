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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.List;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Arrays;

@TeleOp
public class LimelightTestingTeleOp extends LinearOpMode {
    private Hardware hardware;

    private Limelight3A limelight;

    private static double distance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }

    private List<List<double[]>> distances = new ArrayList<>();

    private List<List<double[]>> getDiagonalPairs(List<double[]> corners) {
        double[] reference = corners.get(0);

        for (int i = 1; i < corners.size(); i++) {
            double[] corner = corners.get(i);
            double d = distance(reference[0], reference[1], corner[0], corner[1]);
            List<double[]> pair = new ArrayList<>();
            pair.add(corner);
            pair.add(reference);
            distances.add(pair);
        }

        distances.sort((a, b) -> Double.compare(distance(b.get(0)[0], b.get(0)[1], b.get(1)[0], b.get(1)[1]),
                distance(a.get(0)[0], a.get(0)[1], a.get(1)[0], a.get(1)[1])));

        return distances;
    }

    public List<double[]> getLongPair() {
        return distances.get(1);
    }

    public double getAngle(List<double[]> longPair) {
        double P1x = longPair.get(0)[0];
        double P1y = longPair.get(0)[1];
        double P2x = longPair.get(1)[0];
        double P2y = longPair.get(1)[1];

        double angle = Math.atan2(Math.abs(P2y - P1y), P2x - P1x);
        return Math.toDegrees(angle) - 90;
    }

    public double ServoAngle(double angle) {

        // TODO for Ryan b/c nishk and i will not be here today
        // Servo positioning, clockwise is positive
        // change the clockwise code below and make sure it builds
        // ServoAngle(getAngle(getLongPair()));
        /*if (angle/270 > 0.48) {
            return 0.48+(angle/270);
        }
        return 0.48-(angle/270);*/
        return 0.48+(angle*0.0036);
    }

     @Override
    public void runOpMode() {
        Servo servo = hardwareMap.get(Servo.class, "clawTwist");
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
            hardware.clawTwist.setPosition(ServoAngle(getAngle((getLongPair()))));
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
                telemetry.addData("Pipeline Number: ", limelight.getStatus().getPipelineIndex());

                List<LLResultTypes.ColorResult> colorTargets = result.getColorResults();
                for (LLResultTypes.ColorResult colorTarget : colorTargets) {
                    List<double[]> corners = new ArrayList<double[]>();
                    for(List<Double> eachCorner: colorTarget.getTargetCorners()){
                        if (corners.size() == 4) {
                            break;
                        }
                        telemetry.addData("First Value of Corner", eachCorner);
                        double[] eachCorner2 = eachCorner.stream().mapToDouble(Double::doubleValue).toArray();
                        corners.add(eachCorner2);
                    }
                    getDiagonalPairs(corners);
                    telemetry.addData("Angle: ",getAngle(getLongPair()));
                }
                sleep(1000);
 //

                //telemetry.addData("Avg Dist: ", result.getBotposeAvgDist());
                //telemetry.addData("Botpose", botpose.toString());

            } else {
                telemetry.addData("Limelight", "No data available");
            }

            telemetry.update();

        }
    }
}