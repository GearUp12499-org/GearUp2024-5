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

    private List<List<double[]>> distances = new ArrayList<>(); // [--[ [x, y], [], []... ]--]

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
        hardware = new Hardware(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(3);
        limelight.start();
        hardware.clawTwist.setPosition(Hardware.CLAW_TWIST_INIT);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // TO-DO: Start a timer
            if(gamepad1.a){
                hardware.limelightlight.setPosition(1);
                sleep(500);
                boolean foundAngle = false;
                double angle;
                double pickupPosition = Hardware.CLAW_TWIST_INIT;
                while(!foundAngle) { //TO-DO: set a timer
                    LLResult result = limelight.getLatestResult();

                    if (result != null /* && result.isValid() */) {
                        // Pose3D botpose = result.getBotpose();
                        telemetry.addData("tx", result.getTx());
                        telemetry.addData("ty", result.getTy());
                        telemetry.addData("Avg Area: ", result.getTa());
                        telemetry.addData("Pipeline Number: ", limelight.getStatus().getPipelineIndex());

                        List<LLResultTypes.ColorResult> colorTargets = result.getColorResults();
                        for (LLResultTypes.ColorResult colorTarget : colorTargets) {
                            List<double[]> corners = new ArrayList<double[]>(); //Array of corner pairs
                            for (List<Double> eachCorner : colorTarget.getTargetCorners()) {
                                telemetry.addData("Pair of Corner: ", eachCorner);
                                double[] eachCorner2 = eachCorner.stream().mapToDouble(Double::doubleValue).toArray();
                                corners.add(eachCorner2);

                                if (corners.size() >= 4) {
                                    break;
                                }
                            }
                            if(corners.size() < 3){
                                continue;
                            }
                            getDiagonalPairs(corners);
                            foundAngle = true;
                            angle = getAngle(getLongPair());
                            pickupPosition = (angle * 0.0037) + Hardware.CLAW_TWIST_INIT;
                            telemetry.addData("Angle: ", angle);
                            telemetry.addData("pickupPosition: ", pickupPosition);
                            break;
                        }

                    }
                }
                if(foundAngle) {
                    hardware.clawFront.setPosition(Hardware.FRONT_OPEN);
                    sleep(500);
                    hardware.clawFlip.setPosition(0);
                    sleep(500);
                    hardware.clawTwist.setPosition(pickupPosition);
                    sleep(500);
                    hardware.clawFront.setPosition(Hardware.FRONT_CLOSE);
                    sleep(250);
                    hardware.clawTwist.setPosition(Hardware.CLAW_TWIST_INIT);
                    sleep(100);
                    hardware.clawFlip.setPosition(Hardware.FLIP_UP);
                    hardware.limelightlight.setPosition(0);
                }
            }


            telemetry.update();

            //hardware.clawTwist.setPosition(ServoAngle(getAngle(getLongPair())));
        }
    }
}