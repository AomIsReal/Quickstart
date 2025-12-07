package org.firstinspires.ftc.teamcode.origin;

import static org.firstinspires.ftc.teamcode.origin.Utilize.WrapRads;
import static org.firstinspires.ftc.teamcode.origin.Utilize.toRadian;
import static org.firstinspires.ftc.teamcode.origin.Utilize.toDegree;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Tele", group = "robot")
public  class Tele extends Robot {

    private Controller controller;

    // Variables
    int targetLift = 0;
    double setpoint = 0, H_Ang = 0, AL_Ang = 0, LiftPos = 0;
    boolean  V_Pressed = false, VisBusy = false, ITisOn = false, tp_Pressed = false,
            r_disable = false, R_Pressed = false,RisON = false, ADC_Pressed = false
    ;
    double CurrentTime = System.nanoTime() * 1E-9,  lastRXtime = CurrentTime;

    private void Init() {
        // Initialize Robot
        Initialize();

        controller = new Controller(1.0, 0.05, 0.1, 0 , 0.2, toRadian(0.75), controller.Main_i_max, controller.Main_i_min);

        setpoint = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
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
        double r = r_disable ? 0 : controller.Calculate(WrapRads(setpoint - yaw));
        double x = -gamepad1.right_stick_x;
        if (x != 0 || CurrentTime - lastRXtime < 0.45) {
            r = x;
            setpoint = yaw;
        }
        if (lx == 0 && ly == 0 && x== 0 && Math.abs(r) < 0.2)  r = 0;
        lastRXtime = x != 0 ? CurrentTime : lastRXtime;
        // Denominator for division to get no more than 1
        double d = Math.max(Math.abs(x2) + Math.abs(y2) + Math.abs(r), 0.5);
//        MovePower((y2 - x2 - r) / d, (y2 + x2 + r) / d,
//                (y2 + x2 - r) / d,  (y2 - x2 + r) / d);
        telemetry.addData("yaw", toDegree(yaw));
        telemetry.addData("setpoint", toDegree(setpoint));
        telemetry.addData("error", controller.Error);


    }



    @Override
    public void runOpMode() {
        Init();
        sleep(1000);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                Odomentry();
                Movement();

//                telemetry.addData("XYH", "%6f cm %6f cm", Posx, Posy);
                telemetry.addData("LRM", "%6d  %6d %6d", left_encoder_pos, right_encoder_pos, center_encoder_pos);
                telemetry.addData("heading", toDegree(heading));
                telemetry.addData("XYH", "%6f cm %6f cm", Posx, Posy);
                telemetry.update();
                if(gamepad1.options) {
                    imu.resetYaw();
                    setpoint = 0;
                }
            }

        }
    }
}