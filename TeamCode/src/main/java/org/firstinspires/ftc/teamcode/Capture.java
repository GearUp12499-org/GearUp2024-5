package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.vision.VisionPortal.CameraState.STREAMING;

import android.graphics.Canvas;
import android.os.Environment;
import android.util.Log;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

import java.io.File;
import java.util.concurrent.TimeUnit;

@TeleOp
public class Capture extends LinearOpMode {
    private CaptureProcessor proc;

    static class CaptureProcessor implements VisionProcessor {
        public Mat latest;
        public long frameNo = 0;

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            frameNo++;
            if (latest != null) latest.release();
            latest = frame.clone();
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        }
    }

    private void save(Mat m) {
        int count = 0;
        String pre = (Environment.getExternalStorageDirectory().toString());
        String name = "capture" + count + ".png";
        File f;
        while ((f = new File(pre + "/" + name)).exists()) {
            count++;
            name = "capture" + count + ".png";
        }
        Log.i("Capture", "Saving to " + f.getAbsolutePath());
        Imgcodecs.imwrite(f.getAbsolutePath(), m);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        CameraHardware hw = new CameraHardware(hardwareMap);
        proc = new CaptureProcessor();
        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hw.webcam)
                .setCameraResolution(new Size(1920, 1080))
                .addProcessor(proc)
                .build();

        boolean isAPressed = false;
        boolean configured = false;

        while (opModeInInit()) {
            if (portal.getCameraState() == STREAMING && !configured) {
                ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
                exposure.setMode(ExposureControl.Mode.Manual);
                long micros = exposure.getMinExposure(TimeUnit.MILLISECONDS);
                exposure.setExposure(200, TimeUnit.MILLISECONDS);
                configured = true;
            }
            boolean nonVolatileA = gamepad1.a;
            if (nonVolatileA && !isAPressed) {
                Log.i("Capture", "Saving");
                // try to avoid getting deallocated while saving by cloning
                Mat imm = proc.latest.clone();
                save(imm);
                imm.release();
                Log.i("Capture", "Done");
            }
            if (configured) {

                ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
                telemetry.addData("Exposure (ms)", exposure.getExposure(TimeUnit.MILLISECONDS));
            }
            telemetry.addData("Frame", proc.frameNo);
            telemetry.addData("Push?", isAPressed);
            telemetry.addLine("(press A to save)");
            telemetry.update();

            isAPressed = nonVolatileA;
        }
    }
}
