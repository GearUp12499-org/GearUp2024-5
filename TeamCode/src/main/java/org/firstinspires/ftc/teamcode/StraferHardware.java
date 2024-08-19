package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.teamcode.hardware.HardwareMapper;
import org.firstinspires.ftc.teamcode.hardware.HardwareName;
import org.firstinspires.ftc.teamcode.hardware.Reversed;

public class StraferHardware extends HardwareMapper {
    public StraferHardware(HardwareMap map) {
        super(map);
    }

    @HardwareName
            ("backRight")
    public DcMotor backRight;
    @HardwareName("backLeft")
    @Reversed
    public DcMotor backLeft;
    @HardwareName("frontRight")
    public DcMotor frontRight;
    @HardwareName("frontLeft")
    @Reversed
    public DcMotor frontLeft;
    @HardwareName("distance")
    public Rev2mDistanceSensor distance;
    @HardwareName("gyro")
    public NavxMicroNavigationSensor gyro;
    //public Servo hand;

}
