package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.HardwareMapper;
import org.firstinspires.ftc.teamcode.hardware.HardwareName;

public class CameraHardware extends HardwareMapper {


    @HardwareName("Webcam 1")
    public WebcamName webcam;
    public CameraHardware(HardwareMap map) {
        super(map);
    }
}
