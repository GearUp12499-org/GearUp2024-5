package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.Locks;
import dev.aether.collaborative_multitasking.MultitaskScheduler;
import dev.aether.collaborative_multitasking.ext.While;
import kotlin.jvm.functions.Function0;
@TeleOp
public abstract class ucsdTeleOp extends LinearOpMode {

    private Hardware hardware;
    private MultitaskScheduler scheduler;
    private NavxMicroNavigationSensor navxMicro;
    private double heading = 0.0;

    private While doWhile(Function0<Boolean> condition, Runnable action) {
        return new While(scheduler, condition, action);
    }

    private void hardwareInit() {
        navxMicro = hardware.gyro;
    }

    /////////////////////////////////////////////

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        // this scheduler will only be used in init
        scheduler = new MultitaskScheduler();
        hardware = new Hardware(hardwareMap);
        hardwareInit();

        ElapsedTime timer = new ElapsedTime();

        telemetry.log().add("Calibrating gyroscope. Don't touch...");
        scheduler.add(doWhile(
                navxMicro::isCalibrating,
                () -> {
                    telemetry.addData("Calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
                    telemetry.update();
                }
        ));
        double yaw_offset = 0.0;
        while (opModeIsActive()) {
            telemetry.addLine();


        }
    }
}

