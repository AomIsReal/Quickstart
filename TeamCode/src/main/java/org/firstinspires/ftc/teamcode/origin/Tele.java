package org.firstinspires.ftc.teamcode.origin;

import static org.firstinspires.ftc.teamcode.origin.Utilize.WrapRads;
import static org.firstinspires.ftc.teamcode.origin.Utilize.toRadian;
import static org.firstinspires.ftc.teamcode.origin.Utilize.toDegree;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    double CurrentTime = System.nanoTime() * 1E-9,  lastRXtime = CurrentTime;

    private void Init() {
        // Initialize Robot
        Initialize();

        controller = new Controller(0.95, 0.05, 0.05, 0 , 0.2, toRadian(1), 1.0, -1.0);
        ShooterController = new Controller(0.01, 0.0000000005, 0.01, 0.00049, 0.0, 50, 50.0, -50.0);

        setpoint = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

    }

    private void Movement() {
        CurrentTime = System.nanoTime() * 1E-9;
        double speed = 0.4;
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
        MovePower((y2 - x2 - r) / d, (y2 + x2 + r) / d,
                (y2 + x2 - r) / d,  (y2 - x2 + r) / d);
//        telemetry.addData("yaw", toDegree(yaw));
//        telemetry.addData("setpoint", toDegree(setpoint));
//        telemetry.addData("error", controller.Error);

    }


    @Override
    public void runOpMode() {
        Init();
        sleep(1000);
        while (!isStarted()) {
            if (cam1.getAllDetections().isEmpty()) {
            } else {
                for (AprilTagDetection tag : cam1.getAllDetections()) {
                    if ((tag.id == 20) || (tag.id == 24)) {
                        double bearing = tag.ftcPose.bearing;
                        telemetry.addData("Bearing", bearing);
                        telemetry.update();
                    }
                }
            }
        }
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                Odomentry();
                Movement();
                AutoAim();
                if (gamepad1.right_trigger > 0.2) {
                    InTakeIN();
                }

                if (gamepad1.right_trigger < 0.2) {
                    IT.setPower(0);
                }

                if (gamepad1.right_bumper) {
                    SetServoPos(HoodPos, AG);
                    M = true;

                    if (gamepad1.square) {
                        TargetVelo = 1250;
                        HoodPos = 0.5;
                        SetServoPos(HoodPos, AG);
                        WaitForVelo(TargetVelo, 3, 1, controller);

                    }
                    if (gamepad1.triangle) {
                        TargetVelo = 1500;
                        HoodPos = 0.5;
                    }
                    if (gamepad1.circle) {
                        TargetVelo = 2000;
                        HoodPos = 1.0;
                    }
                }




                if (gamepad1.rightBumperWasReleased()){
                    TargetVelo = 0;
                    HoodPos = 0;
                    SetServoPos(0.85, BubBlebee);
                    M = false;
                }


                if (gamepad1.left_trigger > 0.2) {
                    S = 0.3;
                }
                else S = 1.0;




//                telemetry.addData("LRM", "%6d  %6d %6d", left_encoder_pos, right_encoder_pos, center_encoder_pos);
//                telemetry.addData("heading", toDegree(heading));
//                telemetry.addData("XYH", "%6f cm %6f cm", Posx, Posy);
                telemetry.addData("pid", SL.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
                telemetry.addData("TargetVelo ", TargetVelo);
                telemetry.addData("Current velo ", SR.getVelocity());
                telemetry.update();
                if(gamepad1.options) {
                    imu.resetYaw();
                    setpoint = 0;
                }
            }

        }
    }
}