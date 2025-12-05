package org.firstinspires.ftc.teamcode.origin;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@Configurable
@TeleOp(name = "Shooter Debug Test")
public class ShooterDebugTest extends Robot {

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    public double error;
    public static int TargetVelo = 1500;
    public static double Kp = 0.001;
    public static double Ki = 0.0;
    public static double Kd = 0.000005;
    public static double Kf = 0.000438;

    private Controller controller;

    public void ShooterControl() {
        Dual_SHMotor(Range.clip(controller.Calculate(TargetVelo, SR.getVelocity()),0, 1));
//        SL.setVelocity(TargetVelo);
//        SR.setVelocity(TargetVelo);
    }

    public void PIDF(double P, double I, double D, double Kf) {
        controller = new Controller(P, I, D, Kf, 0.0, 10);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Initialize();
        PIDF(Kp, Ki, Kd, Kf);
        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.a) {
                Dual_SHMotor(0.0);
                TargetVelo = 0;
            }
            if (gamepad1.dpad_up) {
                TargetVelo += 50;
                sleep(50);
            }
            if (gamepad1.dpad_down) {
                TargetVelo -= 50;
                sleep(50);
            }
            ShooterControl();
            if (gamepad1.x) {
                BR.setPower(0.1);
            }
            telemetryM.debug("Shooter Velocity: " + SR.getVelocity());
            telemetryM.debug("Target Velocity: " + TargetVelo);
            telemetryM.update(telemetry);
        }
    }
}
