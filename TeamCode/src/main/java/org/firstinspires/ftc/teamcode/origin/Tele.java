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
    double setpoint = 0;
    boolean  X_Pressed = false, XisBusy = false, T_Pressed = false, TisBusy = false, RB_Pressed = false, RBisBusy = false;

    double CurrentTime = System.nanoTime() * 1E-9,  lastRXtime = CurrentTime;

    private void Init() {
        // Initialize Robot
        Initialize();

        controller = new Controller(0.8, 0.05, 0.05, 0 , 0.2, toRadian(1), 1.0, -1.0);
        ShooterController = new Controller(0.02, 0.0000000001, 0.05, 0.00048, 0.0, 50, 100.0, -100.0);

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
        double x = -gamepad1.right_stick_x / 1.3;
        if (x != 0 || CurrentTime - lastRXtime < 0.45) {
            r = x;
            setpoint = yaw;
        }
        if (lx == 0 && ly == 0 && x== 0 && Math.abs(r) < 0.2)  r = 0;
        lastRXtime = x != 0 ? CurrentTime : lastRXtime;
        // Denominator for division to get no more than 1
        double d = Math.max(Math.abs(x2) + Math.abs(y2) + Math.abs(r), 0.5);
        MovePower((y2 - x2 - r) / d, (y2 + x2 + r) / d,
                (y2 + x2 - r) / d,  (y2 - x2 + r) / d);
        AutoAim();
//        telemetry.addData("yaw", toDegree(yaw));
//        telemetry.addData("setpoint", toDegree(setpoint));
//        telemetry.addData("error", controller.Error);

    }

    public void shoot() {
        boolean sh = gamepad1.cross;
        if (!(sh)) {
            X_Pressed = false;
            return;
        }
        if (X_Pressed) return;
        X_Pressed = true;
        if (!XisBusy) {
            ShooterControl();
            XisBusy = true;
        }
        XisBusy = false;
    }

    public void FarZone() {
        boolean fz = gamepad1.triangle;
        if (!(fz)) {
            T_Pressed = false;
            return;
        }
        if (T_Pressed) return;
        T_Pressed = true;
        if (!TisBusy) {
            SetServoPos(1.0, AG);
            SL.setVelocity(2000);
            SR.setVelocity(2000);
            WaitForVelo();
            TisBusy = true;
        }
        SetServoPos(0.0, AG);
        SetServoPos(0.5, TL, TR);
        Dual_SHMotor(0.0);
        TisBusy = false;
    }

    public void InTake() {
        boolean it = gamepad1.right_bumper;
        if (!(it)) {
            RB_Pressed = false;
            return;
        }
        if (RB_Pressed) return;
        RB_Pressed = true;
        if (!RBisBusy) {
            InTakeIN();
            RBisBusy = true;
        }
        RBisBusy = false;
        IT.setPower(0);
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
                shoot();
                FarZone();
                InTake();
//                telemetry.addData("LRM", "%6d  %6d %6d", left_encoder_pos, right_encoder_pos, center_encoder_pos);
//                telemetry.addData("heading", toDegree(heading));
//                telemetry.addData("XYH", "%6f cm %6f cm", Posx, Posy);
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