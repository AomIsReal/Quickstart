package org.firstinspires.ftc.teamcode.origin;

import static org.firstinspires.ftc.teamcode.origin.Utilize.AtTargetRange;
import static org.firstinspires.ftc.teamcode.origin.Utilize.WrapRads;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public abstract class Robot extends LinearOpMode {
    public webcam cam1;
    public IMU imu;
    public VisionPortal visionPortal;
    public Servo AG, TL, TR , BubBlebee;
    public DcMotorEx  FL, FR, BL, BR, SL, SR, IT, ARM, encoder1, encoder2, encoder3; //
    public int TargetVelo = 0;
    public final double[] tileSize = {60.96, 60.96};
    public final int Counts_Per_Gobilda5000 = 28;
    public double[]       currentXY           = {0, 0};
    public final double   L                   = 20.5; //distance between 1 and 2 in cm
    public final double   B                   = 8.5; //distance between center of 1 and 2 and 3 in cm
    public final double   r                   = 2.4 ; // Odomentry wheel radius in cm
    public final double   N                   = 2000.0 ; // ticks per one rotation
    public double         cm_per_tick     = 2.0 * Math.PI * r / N ;
    public double         TurPos = 0.5, HoodPos = 0.0;
    public double         theta, Posx, Posy, heading, dyaw;
    public final int Chalee = 40;
    //    // update encoder
    int                   left_encoder_pos , right_encoder_pos , center_encoder_pos ,
            prev_left_encoder_pos, prev_right_encoder_pos, prev_center_encoder_pos = 0;
    double                CurrentYaw, OldYaw         = 0;
    private final double Current_Time = System.nanoTime() * 1E-9;
    private  double Last_Time = Current_Time;
    private double Last_yaw;
    public boolean M, sho = false;
    public ElapsedTime runtime = new ElapsedTime();

    @IgnoreConfigurable
    public static TelemetryManager telemetryM;

    private Controller controller;
    public Controller ShooterController;

    public void Initialize() {
        imu = hardwareMap.get(IMU.class,       "imu");
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        cam1 = new webcam() {};
        cam1.init(hardwareMap, telemetryM);
        FL  = hardwareMap.get(DcMotorEx.class, "FL");    FR  = hardwareMap.get(DcMotorEx.class, "FR");
        BL  = hardwareMap.get(DcMotorEx.class, "BL");    BR  = hardwareMap.get(DcMotorEx.class, "BR");
        SL  = hardwareMap.get(DcMotorEx.class, "SL");    SR  = hardwareMap.get(DcMotorEx.class, "SR");
        IT  = hardwareMap.get(DcMotorEx.class, "IT");
        ARM  = hardwareMap.get(DcMotorEx.class, "ARM");
        AG  = hardwareMap.get(Servo.class,     "AG");    TL  = hardwareMap.get(Servo.class,     "TL");
        TR  = hardwareMap.get(Servo.class,     "TR");    BubBlebee  = hardwareMap.get(Servo.class,     "BubBlebee");


        Last_yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        encoder1 = BL;
        encoder2 = FR;
        encoder3 = FL;
        Posy = 0;
        Posx = 0;

        // Initialize IMU
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection .UP)));
        sleep(50);
        // Reverse Servo

        // setMode Motors
        SL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ARM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        SR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        IT.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        ARM.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SL.setDirection(DcMotor.Direction.FORWARD);
        SR.setDirection(DcMotor.Direction.REVERSE);

        FL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.REVERSE);

        IT.setDirection(DcMotor.Direction.FORWARD);
        ARM.setDirection(DcMotor.Direction.FORWARD);

        // SetBehavior Motors
        SL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ARM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        SetServoPos(0.5, TL, TR);
        SetServoPos(0, AG);
        SetServoPos(0.85, BubBlebee);
        sleep(20);
    }
    public void Dual_SHMotor() {
//        SL.setPower(Sh_con());
//        SR.setPower(Sh_con());
        SL.setVelocity(TargetVelo);
        SR.setVelocity(TargetVelo);
    }

    public double Sh_con() {
        double Power = ShooterController.Calculate(TargetVelo, SR.getVelocity());
        Power = Range.clip(Power, 0, 1);
        return Power;
    }

    public void Odomentry() {
        left_encoder_pos = encoder1.getCurrentPosition();
        right_encoder_pos = -encoder2.getCurrentPosition();
        center_encoder_pos = encoder3.getCurrentPosition();

        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        heading = yaw;

        double delta_left_encoder_pos = (left_encoder_pos - prev_left_encoder_pos) * cm_per_tick;
        double delta_right_encoder_pos = (right_encoder_pos - prev_right_encoder_pos) * cm_per_tick;
        double delta_center_encoder_pos = (center_encoder_pos - prev_center_encoder_pos) * cm_per_tick;

//        double phi = (delta_right_encoder_pos - delta_left_encoder_pos) / L;
        double phi = WrapRads(Last_yaw - yaw);
//        telemetry.addData("phi", phi);
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
    public void AutoAim() {
        if (cam1.getAllDetections().isEmpty()) {
            } else {
                for (AprilTagDetection tag : cam1.getAllDetections()) {
                    if ((tag.id == 20) || (tag.id == 24)) {
                        double bearing = tag.ftcPose.bearing + 3.173;
                        if (Math.abs(bearing) > 1.2) {

                            TurPos += bearing * 0.0007;

                            // Safety: Ensure we don't tell the servo to go past physical limits
                            TurPos = Math.max(0.0, Math.min(1.0, TurPos));
                        }


                        TurPos = Range.clip(TurPos, 0, 1);
                        SetServoPos(TurPos, TL, TR);
                        telemetry.addData("id", tag.id);
                        telemetry.addData("Range", tag.ftcPose.range);
                        telemetry.update();
//                    SetServoPos(TurPos, TL, TR);
                    }
                }
            }
//        SetServoPos(HoodPos, AG);
    }


//    public void WaitForVelo(int Tvelo, int nloop, double EndLoopTime, Controller controller){
//        int check_num = 2000000000,check = 0;
//        TargetVelo = Tvelo;
//        Dual_SHMotor();
//        for (int n = 0; n < nloop; n++){
//            double LoopTime = System.nanoTime() * 1E-9;
//            while (true) {
////                AutoAim();
//                if (AtTargetRange(SR.getVelocity(), Tvelo, Chalee)) check++;
//                else check = 0;
//                if (check >= check_num || (System.nanoTime() * 1E-9 - LoopTime < EndLoopTime)) {
//                    SetServoPos(0.92, BubBlebee);
//                    IT.setPower(-1);
//                    sleep(500);
//                    IT.setPower(0);
//                    check =0;
//                    if (controller == null) break;
//                    else controller.Reset();
//
//
//                    break;
//                }
//            }
//        }
//    }

    public void WaitForVelo(int Tvelo, int nloop, double EndLoopTime, Controller controller, boolean isAutoAim){
        TargetVelo = Tvelo;
        Dual_SHMotor();
//        ServoController.PwmStatus.DISABLED.name("BubBlebee");
        while (!AtTargetRange(SR.getVelocity(), Tvelo, Chalee));
        for (int n = 0; n < nloop; n++){
            double loopStart = System.nanoTime() * 1E-9;
            double checkStartTime = loopStart;
            int stableCount = 0;
            int requiredStableCount = 4;
            while (true) {
                if (isAutoAim) AutoAim();
                // เปลี่ยนเป็นเช็คเวลาให้เป็นหลัก — หยุดเมื่อเกิน EndLoopTime
                double elapsed = System.nanoTime() * 1E-9 - loopStart;
                if (AtTargetRange(SR.getVelocity(), Tvelo, Chalee)) {
                    stableCount++;
                } else {
                    stableCount = 0;
                }

                if (stableCount >= requiredStableCount) {
                    // คิดว่า stable แล้ว ให้ทำ action แล้วออก
                    SetServoPos(0.9, BubBlebee);
//                    if (nloop == 2){
//                        SetServoPos(0.915, BubBlebee);
//                    }
                    IT.setPower(-1);
                    sleep(500);
                    IT.setPower(0);
                    if (controller != null) controller.Reset();
                    break;
                }

                if (elapsed > EndLoopTime) {
                    // เวลาหมด ให้ทำ actionแล้วออก
                    SetServoPos(0.9, BubBlebee);
//                    if (nloop == 2){
//                        SetServoPos(0.915, BubBlebee);
//                    }
                    IT.setPower(-1);
                    sleep(500);
                    IT.setPower(0);
                    if (controller != null) controller.Reset();
                    break;
                }
            }
        }
    }

    public void InTakeIN(){
        if (!(M)) {
            SetServoPos(0.85, BubBlebee);
        }
        IT.setPower(-1);

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
