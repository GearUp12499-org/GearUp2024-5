package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Ascent;
import org.firstinspires.ftc.teamcode.hardware.AutoClearEncoder;
import org.firstinspires.ftc.teamcode.hardware.Encoder;
import org.firstinspires.ftc.teamcode.hardware.EncoderFor;
import org.firstinspires.ftc.teamcode.hardware.HardwareMapper;
import org.firstinspires.ftc.teamcode.hardware.HardwareName;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.MotorSet;
import org.firstinspires.ftc.teamcode.hardware.Reversed;
import org.firstinspires.ftc.teamcode.hardware.ZeroPower;
import org.firstinspires.ftc.teamcode.mmooover.TriOdoProvider;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.Servo;

import dev.aether.collaborative_multitasking.SharedResource;


public class Hardware extends HardwareMapper implements TriOdoProvider {
    public static final int ARM_TRANSFER_POS = -40;
    public static final double SCORE_SPECIMEN_ARM_DEG =-100;
    public static final double LEFT_ARM_SCORE = 0.44;
    public static final double RIGHT_ARM_SCORE = 1 - LEFT_ARM_SCORE;
    public static final double LEFT_ARM_TRANSFER = 0.05;
    public static final double RIGHT_ARM_TRANSFER = 1 - LEFT_ARM_TRANSFER;
    public static final double spinTickPerRev = 751.8;
    public static final double RIGHT_SLIDE_OUT = 0.65;
    @Deprecated public static final double LEFT_SLIDE_OUT = 1.05 - RIGHT_SLIDE_OUT;
    public static final double RIGHT_SLIDE_IN = 0.34;
    @Deprecated public static final double LEFT_SLIDE_IN = 1.05 - RIGHT_SLIDE_IN;
    public static final double RIGHT_SLIDE_TRANSFER = 0.4;
    public static final double LEFT_SLIDE_TRANSFER = 1.05 - RIGHT_SLIDE_TRANSFER;
    public static final double CLAW_TWIST_INIT = 0.48;
    public static final double CLAW_TWIST_MAX = 0.82;
    public static final double CLAW_TWIST_MIN = 0.13;
    public static final double SLIDE_INWARD_TIME = 0.75; // seconds
    public static final double SLIDE_OUTWARD_TIME = 0.45; // seconds
    public static final double SLIDE_OVERSHOOT = 0.28;
    public static final double FLIP_DOWN = 0.2;
    public static final double FRONT_OPEN = 0.66;
    public static final double FRONT_CLOSE = 0.37;
    public static final double FLIP_UP = 0.8;
    public static final double FLIP_ONE_THIRD = 0.4;
    public static final double CLAW_CLOSE = 0.8;
    public static final double CLAW_OPEN = 0.46;
    public static final double WRIST_BACK = 0.3;
    public static final double WRIST_UP = 0.36;
    public static final double WRIST_SCORE = 0.89;
    public static final double WRIST_TRANSFER = 0.37;
    public static final double ARM_POWER = 0.2;
    public static final double LAMP_BLUE = 0.611;
    public static final double LAMP_RED = 0.28;
    public static final double LAMP_ORANGE = 0.333;
    public static final double LAMP_YELLOW = 0.36;
    public static final double LAMP_PURPLE = 0.700;
    public static final double LAMP_GREEN = 0.440;
    public static final int VLIFT_MAX_HEIGHT = 825;
    public static final int VLIFT_SCORE_HIGH = 790;
    public static final int VLIFT_SCORE_SPECIMEN = 283;
    public static final double VLIFT_CLOSENESS = 50.0;
    public static final int VLIFT_POWEROFF_HEIGHT = 30;
    public static final int ASCENT_INIT_POS = -500;
    public static final int ASCENT_UP_POS = -1742;
    public static final int ASCENT_PREPARE_POS = -2995;
    public static final int ASCENT_FINISH_POS = -50;


    public static int deg2arm(double degrees) {
        return (int) (degrees / 360.0 * spinTickPerRev);
    }

    public static class Locks {
        /// The four drive motors: frontLeft, frontRight, backLeft, and backRight.
        public static final SharedResource DriveMotors = new SharedResource("DriveMotors");

        /// The vertical slide.
        ///
        /// It usually makes sense to have one task 'own' this the entire time
        /// and provide its own APIs and lock.
        public static final SharedResource VerticalSlide = new SharedResource("VerticalSlide");

        /// The components that make up the main arm assembly:
        /// * the `arm` motor
        /// * the `wrist` and `claw` servos
        public static final SharedResource ArmAssembly = new SharedResource("ArmAssembly");

        /// The `horizontalRight` and `horizontalLeft` servos.
        public static final SharedResource horizontalRight = new SharedResource("horizontalRight");

        public static final SharedResource HSlideClaw = new SharedResource("HSlideClaw");

        public static final SharedResource Ascent = new SharedResource("Ascent");

        /// The two lights on the top of the robot.
        public static final SharedResource Signals = new SharedResource("Signals");

        /// Limelight 3A.
        public static final SharedResource Limelight = new SharedResource("Limelight");
    }

    public static final double TRACK_WIDTH = 11.375;
    public static final double FORWARD_OFFSET = 3.062500;
    public static final double ENC_WHEEL_RADIUS = 1.25984 / 2.0;
    public static final int ENC_TICKS_PER_REV = 2000;

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

    @HardwareName("verticalSlides")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    private DcMotor verticalSlide;

    @HardwareName("verticalSlide2")
    @Reversed
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    private DcMotor verticalSlide2;

    public Lift verticalLift;

    @EncoderFor("verticalSlides")
    @AutoClearEncoder
    public Encoder encoderVerticalSlide;

//    @HardwareName("arm")
//    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
//    @AutoClearEncoder
//    public DcMotor arm;

    @EncoderFor("frontLeft")
    @AutoClearEncoder
    public Encoder encoderLeft;

    @EncoderFor("backRight")
    @AutoClearEncoder
    @Reversed
    public Encoder encoderCenter;

    @EncoderFor("frontRight")
    @AutoClearEncoder
    @Reversed
    public Encoder encoderRight;

    @HardwareName("gyro")
    public NavxMicroNavigationSensor gyro;

    @HardwareName("claw")
    public Servo claw;

    @HardwareName("wrist")
    public Servo wrist;

    @HardwareName("clawFront")
    public Servo clawFront;

    @HardwareName("rightFlip")
    public Servo rightFlip;

    @HardwareName("armLeft")
    public Servo armLeft;

    @HardwareName("armRight")
    public Servo armRight;

    @HardwareName("clawTwist")
    public Servo clawTwist;

    @HardwareName("horizontalRight")
    public Servo horizontalRight;

    @HardwareName("horizontalLeft")
    public Servo horizontalLeft;

    @HardwareName("colorLeft")
    public Servo colorLeft;

    @HardwareName("colorRight")
    public Servo colorRight;

    @HardwareName("lightLeft")
    public Servo lightLeft;

    @HardwareName("lightRight")
    public Servo lightRight;

    @HardwareName("leftFlip")
    public Servo leftFlip;


    @HardwareName("clawColor")
    public ColorSensor clawColor;

    @EncoderFor("backLeft")
    @Reversed
    @AutoClearEncoder
    public Encoder leftAscentEnc;

    //@HardwareName("leftAscent")
    //public CRServo leftAscent;

    @EncoderFor("verticalSlide2")
    @AutoClearEncoder
    public Encoder rightAscentEnc;

    //@HardwareName("rightAscent")
    //@Reversed
    //public CRServo rightAscent;

    public Ascent ascent;

    @HardwareName("limelight")
    public Limelight3A limelight;

    @Override
    public Encoder getLeftEncoder() {
        return encoderLeft;
    }

    @Override
    public Encoder getRightEncoder() {
        return encoderRight;
    }

    @Override
    public Encoder getCenterEncoder() {
        return encoderCenter;
    }
    // Values calculated from [very scuffed] CAD measurements.


    @Override
    public double getTrackWidth() {
        return TRACK_WIDTH;
    }

    @Override
    public double getForwardOffset() {
        return FORWARD_OFFSET;
    }

    @Override
    public double getEncoderWheelRadius() {
        return ENC_WHEEL_RADIUS;
    }

    @Override
    public int getEncoderTicksPerRevolution() {
        return ENC_TICKS_PER_REV;
    }

    public MotorSet driveMotors;

    public void sharedHardwareInit() {
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFlip.setPosition(Hardware.FLIP_UP);
        clawFront.setPosition(Hardware.FRONT_OPEN);
        clawTwist.setPosition(Hardware.CLAW_TWIST_INIT);

//        arm.setTargetPosition(0);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        arm.setPower(0.3);
        //wrist.setPosition(0.28);
        claw.setPosition(Hardware.CLAW_CLOSE);

        // we don't have the proxy object to handle this for us
        // so manually implement the inversion
        horizontalRight.setPosition(Hardware.RIGHT_SLIDE_IN);
        horizontalLeft.setPosition(1.05 - Hardware.RIGHT_SLIDE_IN);

        colorLeft.setPosition(Hardware.LAMP_PURPLE);
        colorRight.setPosition(Hardware.LAMP_PURPLE);

        limelight.stop();
    }

    public Hardware(HardwareMap hwMap) {
        super(hwMap);
        driveMotors = new MotorSet(
                frontLeft,
                frontRight,
                backLeft,
                backRight
        );
        verticalLift = new Lift(verticalSlide, verticalSlide2);
//        ascent = new Ascent(leftAscent, leftAscentEnc, rightAscent, rightAscentEnc);
        ascent = null;
    }

}