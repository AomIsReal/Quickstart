package org.firstinspires.ftc.teamcode.origin;

import android.widget.ToggleButton;

import static org.firstinspires.ftc.teamcode.origin.Utilize.WrapRads;
import static org.firstinspires.ftc.teamcode.origin.Utilize.toRadian;
import static org.firstinspires.ftc.teamcode.origin.Utilize.toDegree;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="Tele", group = "robot")
public  class Tele extends Robot {

    private Controller controller;


    // Variables
    String Status;
    double setpoint = 0, S = 1;
    boolean  PS_Pressed = false, PSisBusy = false, T_Pressed = false, TisBusy = false, RB_Pressed = false, RBisBusy = false ;
    boolean circlePress = false;
    boolean circleState = false; //
    boolean xPress = false;
    boolean xState = false;

    long lastTime = 0;
    boolean servoPos = false;
    final long INTERVAL = 1200; // 1 วิ
    double CurrentTime = System.nanoTime() * 1E-9,  lastRXtime = CurrentTime;

    private void Init() {
        // Initialize Robot
        Initialize();

        controller = new Controller(1.00, 0.05, 0.05, 0 , 0.2, toRadian(1), 1.0, -1.0);
        ShooterController = new Controller(0.01, 0.0000000005, 0.01, 0.00049, 0.0, 50, 50.0, -50.0);

        setpoint = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

    }

    private void Movement() {
        CurrentTime = System.nanoTime() * 1E-9;
        double speed = 0.1; //0.3
        double lx = -gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double x1 = gamepad1.dpad_right ? -speed : gamepad1.dpad_left ? speed : lx;
        double y1 = gamepad1.dpad_up ? speed : gamepad1.dpad_down ? -speed : ly;
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double x2 = (Math.cos(yaw) * x1) - (Math.sin(yaw) * y1);
        double y2 = (Math.sin(yaw) * x1) + (Math.cos(yaw) * y1);
        // Rotate
        double r = controller.Calculate(WrapRads(setpoint - yaw));
        double x = -gamepad1.right_stick_x * S;
        if (x != 0 || CurrentTime - lastRXtime < 0.45) {
            r = x;
            setpoint = yaw;
        }
        if (Math.abs(lx) <= 0.05 && Math.abs(ly) <= 0.05 && Math.abs(x) <= 0.05 && Math.abs(r) <= 0.2)  r = 0;
        lastRXtime = x != 0 ? CurrentTime : lastRXtime;
        // Denominator for division to get no more than 1
        double d = Math.max(Math.abs(x2) + Math.abs(y2) + Math.abs(r), 0.5);
        MovePower((y2 + x2 - r) / d, (y2 - x2 + r) / d,
                (y2 - x2 - r) / d,  (y2 + x2 + r) / d);
        telemetry.addData("yaw", toDegree(yaw));
        telemetry.addData("setpoint", toDegree(setpoint));
        telemetry.addData("error", controller.Error);

    }

    private void RunServoLoop() {

        if (!circleState) return;

        if (System.currentTimeMillis() - lastTime >= INTERVAL) {
            lastTime = System.currentTimeMillis();

            servoPos = !servoPos;

            if (servoPos) {
                SetServoPos(0.17, ser1);
            } else {
                SetServoPos(-0.10, ser1);
            }
        }
    }
    private void ToggleX(boolean button) {

        if (!button) {
            xPress = false;
            return;
        }

        if (xPress) return;
        xPress = true;

        xState = !xState;

        if (xState) {
            SetServoPos(0.70, ser2);
        } else {
            SetServoPos(0.40, ser2);
        }
    }
    private void ToggleCircle(boolean button) {

        if (!button) {
            circlePress = false;
            return;
        }

        if (circlePress) return;
        circlePress = true;

        circleState = !circleState;

        // สำคัญมาก: ตอนปิด สั่งหยุดครั้งเดียว
        if (!circleState) {
            SetServoPos(0.12, ser1);
            servoPos = false;
        }
    }

    /*private void ToggleButton(boolean button) {

        if (!button) {
            press = false;
            return;
        }

        if (press) return;
        press = true;

        if (!state) {
            // Action 1
            SetServoPos(0.15 , ser1);
            //SetServoPos(0.5 , ser2);
            sleep(500);
            SetServoPos(-0.15 , ser1);
            //SetServoPos(0 , ser2);
            state = true;
            return;
        }

        // Action 2
        SetServoPos(0 , ser1);
        //SetServoPos(0 , ser2);
        state = false;
    }*/

    /*private void Toggle3(boolean button) {

        if (!button) {
            press = false;
            return;
        }

        if (press) return;
        press = true;

        if (!state) {
            // Action 1
            SetServoPos(0.1 , ser3);
            state = true;
            return;
        }

        // Action 2
        SetServoPos(0 , ser3);
        state = false;
    }*/




    @Override
    public void runOpMode() {
        Init();
        sleep(1000);
//        while (!isStarted()) {
//            if (cam1.getAllDetections().isEmpty()) {
//            } else {
//                for (AprilTagDetection tag : cam1.getAllDetections()) {
//                    if ((tag.id == 20) || (tag.id == 24)) {
//                        double bearing = tag.ftcPose.bearing;
//                        telemetry.addData("Bearing", bearing);
//                        telemetry.update();
//                    }
//                }
//            }
//        }

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                Movement();
                ToggleCircle(gamepad1.circle);
                ToggleX(gamepad1.square);
                //Toggle3(gamepad1.square);
                RunServoLoop();


                }
            }

        }
    }
