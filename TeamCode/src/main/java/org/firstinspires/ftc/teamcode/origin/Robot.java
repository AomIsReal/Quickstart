package org.firstinspires.ftc.teamcode.origin;

import static org.firstinspires.ftc.teamcode.origin.Utilize.WrapRads;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;

public abstract class Robot extends LinearOpMode {
    public IMU imu;
    public VisionPortal visionPortal;
    public Servo AG ;
    public DcMotorEx FL, FR, BL, BR, SL, SR, encoder1, encoder2, encoder3;
    public int TargetVelo = 0;
    public final double[] tileSize = {60.96, 60.96};
    public final int Counts_Per_Gobilda5000 = 28;
    public final int Counts_per_HD_HEX      = 28;
    public double[]       currentXY           = {0, 0};
    public final double   L                   = 33.4; //distance between 1 and 2 in cm
    public final double   B                   = 9.65; //distance between center of 1 and 2 and 3 in cm
    public final double   r                   = 2.4 ; // Odomentry wheel radius in cm
    public final double   N                   = 2000.0 ; // ticks per one rotation
    public double         cm_per_tick     = 2.0 * Math.PI * r / N ;
    public double         theta, Posx, Posy, heading, n, CurPosLift, Lift_Power, dn1, dn2, dn3, dyaw ;
    //    // update encoder
    int                   left_encoder_pos , right_encoder_pos , center_encoder_pos ,
            prev_left_encoder_pos, prev_right_encoder_pos, prev_center_encoder_pos = 0;
    double                CurrentYaw, OldYaw         = 0;
    private final double Current_Time = System.nanoTime() * 1E-9;
    private  double Last_Time = Current_Time;
    private double Last_yaw;


    public ElapsedTime runtime = new ElapsedTime();

    public void Initialize() {
        imu = hardwareMap.get(IMU.class,       "imu");
        FL  = hardwareMap.get(DcMotorEx.class, "FL");    FR  = hardwareMap.get(DcMotorEx.class, "FR");
        BL  = hardwareMap.get(DcMotorEx.class, "BL");    BR  = hardwareMap.get(DcMotorEx.class, "BR");
        SL  = hardwareMap.get(DcMotorEx.class, "SL");    SR  = hardwareMap.get(DcMotorEx.class, "SR");

        // Initialize IMU
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection .RIGHT)));
        // Reverse Servo

        // setMode Motors
        SL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        SR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // SetBehavior Motors
        SL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        SR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }
    public void Dual_SHMotor(double Power) {
        SL.setPower(Power);
        SR.setPower(Power);
    }

    public void Odomentry() {
        left_encoder_pos = -encoder1.getCurrentPosition();
        right_encoder_pos = -encoder2.getCurrentPosition();
        center_encoder_pos = encoder3.getCurrentPosition();

        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        heading = yaw;

        double delta_left_encoder_pos = (left_encoder_pos - prev_left_encoder_pos) * cm_per_tick;
        double delta_right_encoder_pos = (right_encoder_pos - prev_right_encoder_pos) * cm_per_tick;
        double delta_center_encoder_pos = (center_encoder_pos - prev_center_encoder_pos) * cm_per_tick;

//        double phi = (delta_right_encoder_pos - delta_left_encoder_pos) / L;
        double phi = WrapRads(Last_yaw - yaw);
        telemetry.addData("phi", phi);
        double delta_middle_pos = (delta_left_encoder_pos + delta_right_encoder_pos) / 2.0;
        double delta_perp_pos = delta_center_encoder_pos - B * phi;

        double delta_x = delta_perp_pos * Math.cos(heading) - delta_middle_pos * Math.sin(heading);
        double delta_y = delta_perp_pos * Math.sin(heading) + delta_middle_pos * Math.cos(heading);

        Posx += delta_x;
        Posy += delta_y;
//        heading += phi;

//        heading = WrapRads(heading);

        prev_left_encoder_pos = left_encoder_pos;
        prev_right_encoder_pos = right_encoder_pos;
        prev_center_encoder_pos = center_encoder_pos;
        Last_yaw = yaw;

    }
    public void MovePower(double Front_Left, double Front_Right,
                          double Back_Left,  double Back_Right) {
        FL.setPower(Front_Left);
        FR.setPower(Front_Right);
        BL.setPower(Back_Left);
        BR.setPower(Back_Right);
    }
    public void Break(double stopSecond) {
        if (stopSecond == 0) return;
        MovePower(0, 0, 0, 0);
        MoveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep((long) (stopSecond * 1000));
    }

    public void MoveMode(DcMotor.RunMode moveMode) {
        FL.setMode(moveMode);
        FR.setMode(moveMode);
        BL.setMode(moveMode);
        BR.setMode(moveMode);
    }

    public double SetServoPos(double pos, float[] minMax, Servo L_servo, Servo R_servo) {
        pos = Range.clip(pos, minMax[0], minMax[1]);
        L_servo.setPosition(pos);
        R_servo.setPosition(pos);
        return pos;
    }

    public double SetServoPos(double pos, Servo L_servo, Servo R_servo) {
        pos = Range.clip(pos, 0, 1);
        L_servo.setPosition(pos);
        R_servo.setPosition(pos);
        return pos;
    }

    public double SetServoPos(double pos, float[] minMax, Servo servo){
        pos = Range.clip(pos, minMax[0], minMax[1]);
        servo.setPosition(pos);
        return pos;
    }

    public double SetServoPos(double pos, Servo servo){
        pos = Range.clip(pos, 0, 1);
        servo.setPosition(pos);
        return pos;
    }
}
