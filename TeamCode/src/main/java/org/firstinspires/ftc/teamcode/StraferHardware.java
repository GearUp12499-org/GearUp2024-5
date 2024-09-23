package org.firstinspires.ftc.teamcode;

import android.graphics.Camera;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.hardware.AutoClearEncoder;
import org.firstinspires.ftc.teamcode.hardware.HardwareMapper;
import org.firstinspires.ftc.teamcode.hardware.HardwareName;
import org.firstinspires.ftc.teamcode.hardware.Reversed;
import org.firstinspires.ftc.teamcode.hardware.ZeroPower;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class StraferHardware extends HardwareMapper {
    public StraferHardware(HardwareMap map) {
        super(map);
    }

    @HardwareName("hand")
    public Servo hand;

    @HardwareName("backRight")
    @AutoClearEncoder
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotor backRight;

    @HardwareName("backLeft")
    @AutoClearEncoder
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    @Reversed
    public DcMotor backLeft;

    @HardwareName("frontRight")
    @AutoClearEncoder
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotor frontRight;

    @HardwareName("frontLeft")
    @AutoClearEncoder
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    @Reversed
    public DcMotor frontLeft;

    @HardwareName("distance")
    public Rev2mDistanceSensor distance;

    @HardwareName("gyro")
    public NavxMicroNavigationSensor gyro;

    @HardwareName("Webcam 1")
    public CameraName Webcam1;

    @HardwareName("ribbit")
    public ColorSensor ribbit;
  //  public Servo hand;

}
//end class.
