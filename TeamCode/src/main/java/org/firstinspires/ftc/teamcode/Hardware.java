package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.AutoClearEncoder;
import org.firstinspires.ftc.teamcode.hardware.HardwareMapper;
import org.firstinspires.ftc.teamcode.hardware.HardwareName;
import org.firstinspires.ftc.teamcode.hardware.MotorSet;
import org.firstinspires.ftc.teamcode.hardware.Reversed;
import org.firstinspires.ftc.teamcode.hardware.ZeroPower;
import org.firstinspires.ftc.teamcode.mmooover.TriOdoProvider;

import java.util.List;

public class Hardware extends HardwareMapper implements TriOdoProvider {
    // left = left motor = exp 0 frontLeft
    // right = right motor = ctr 0 frontRight
    // center = ctr 3 intake

    @HardwareName("frontLeft")
    @Reversed
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotor frontLeft;

    @HardwareName("frontRight")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotor frontRight;

    @HardwareName("backLeft")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    @Reversed
    public DcMotor backLeft;

    @HardwareName("backRight")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotor backRight;

    @HardwareName("frontLeft")
    @AutoClearEncoder
    public DcMotor encoderLeft;

    @HardwareName("intake")
    @AutoClearEncoder
    public DcMotor encoderCenter;

    @HardwareName("frontRight")
    @AutoClearEncoder
    public DcMotor encoderRight;

    @Override
    public DcMotor getLeftEncoder() {
        return encoderLeft;
    }

    @Override
    public DcMotor getRightEncoder() {
        return encoderRight;
    }

    @Override
    public DcMotor getCenterEncoder() {
        return encoderCenter;
    }

    @Override
    public double getTrackWidth() {
        return 14 + 7 / 16.;
    }

    @Override
    public double getForwardOffset() {
        return -(6 + 3 / 4.);
    }

    public MotorSet driveMotors;
    public final boolean hasBulkReads;
    private List<LynxModule> lynxModules = null;

    public void clearCache() {
        if (!hasBulkReads) {
            Log.w("Hardware", "Bulk reads are not enabled, so clearing the cache does nothing!");
            return;
        }
        for (LynxModule hub : lynxModules) hub.clearBulkCache();
    }

    public Hardware(HardwareMap hwMap) {
        this(hwMap, false);
    }

    public Hardware(HardwareMap hwMap, boolean withBulkReads) {
        super(hwMap);
        hasBulkReads = withBulkReads;
        if (withBulkReads) {
            lynxModules = hwMap.getAll(LynxModule.class);
            for (LynxModule ctrl : lynxModules) {
                ctrl.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
        }
        driveMotors = new MotorSet(
                frontLeft,
                frontRight,
                backLeft,
                backRight
        );
    }
}
