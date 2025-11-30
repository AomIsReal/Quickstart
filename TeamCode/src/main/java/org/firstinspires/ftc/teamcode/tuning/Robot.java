package org.firstinspires.ftc.teamcode.tuning;

import static java.lang.Math.abs;
import static java.lang.Math.clamp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.VisionPortal;

public abstract class Robot extends LinearOpMode {
    public IMU imu;
    public VisionPortal visionPortal;
    public Servo AG ;
    public DcMotorEx FL, FR, BL, BR; //SL, SR
    public int TargetVelo = 0;
    public final double[] tileSize = {60.96, 60.96};
    public final int Counts_Per_Gobilda5000 = 28;
    private final double Current_Time = System.nanoTime() * 1E-9;
    private double Last_Time = Current_Time;

    public ElapsedTime runtime = new ElapsedTime();

    public void Initialize() {
        imu = hardwareMap.get(IMU.class,       "imu");
        FL  = hardwareMap.get(DcMotorEx.class, "FL");    FR  = hardwareMap.get(DcMotorEx.class, "FR");
        BL  = hardwareMap.get(DcMotorEx.class, "BL");    BR  = hardwareMap.get(DcMotorEx.class, "BR");
//        SL  = hardwareMap.get(DcMotorEx.class, "SL");    SR  = hardwareMap.get(DcMotorEx.class, "SR");

        // Initialize IMU
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection .RIGHT)));
        // Reverse Servo

        // setMode Motors
//        SL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        SR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        SL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        SR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // SetBehavior Motors
//        SL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        SR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }
    public void Dual_SHMotor(double Power) {
//        SL.setPower(Power);
//        SR.setPower(Power);
    }
}
