package org.firstinspires.ftc.teamcode.origin;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.origin.ShooterDebugTest.follower;

//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Configurable
@TeleOp(name = "Shooter Debug Test")
public class ShooterDebugTest extends Robot {

    // Panels adjustable parameters
    public static int TargetVelo = 1500;
    public static double Kp = 0.02;
    public static double Ki = 0.0000000001;
    public static double Kd = 0.05;
    public static double Kf = 0.00048;
    public static double i_max = 1.0;
    public static double i_min = -1.0;
    private double servoPos = 0.5;

    // Non-editable internal values
    @IgnoreConfigurable
    private Controller ShooterController;
    private Controller TurretController;
    public static Follower follower;

    /**
     * FIX THE CRASH:
     * Panels internally expects a follower, but your shooter test does not use pathing.
     * So we give Panels a dummy follower that always returns a valid pose (0,0,0).
     */
    private void supplyDummyFollowerForPanels() {
        if (follower == null) {
            follower = Constants.createFollower(hardwareMap);
            PanelsConfigurables.INSTANCE.refreshClass(this);
        } else {
            follower = Constants.createFollower(hardwareMap);
        }

        follower.setStartingPose(new Pose());
    }

    // PIDF wrapper for your shooter controller
    public void PIDF(double[] P, double[] I, double[] D, double F) {
        ShooterController = new Controller(P[0], I[0], D[0], F, 0.0, 10, i_max, i_min);
        TurretController = new Controller(P[1], I[1], D[1], 0, 0.0, 2, 1.0, -1.0);
    }

    public void ShooterControl() {
        double power = ShooterController.Calculate(TargetVelo, SR.getVelocity());
//        Dual_SHMotor(Range.clip(power, 0, 1));
//        Dual_SHMotor(TargetVelo);

    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Prevent Pedro/Panels crash
        supplyDummyFollowerForPanels();
        // Normal robot init
        Initialize();
        // Create PIDF controller
        PIDF(new double[]{Kp, 0.002}, new double[]{Ki, 0.0001}, new double[]{Kd, 0.0001}, Kf);
        telemetryM.addData("servoPos", servoPos);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryM.update(telemetry);

        waitForStart();
        while (opModeIsActive()) {
            // Manual control for testing
            if (gamepad1.a) {
                ShooterControl();
                TargetVelo = 0;
            }

            if (gamepad1.dpad_up) {
                TargetVelo += 50;
                sleep(70);
            }

            if (gamepad1.dpad_down) {
                TargetVelo -= 50;
                sleep(70);
            }



//            AutoAim();
            SetServoPos(TurPos, TL, TR);
            // Apply PIDF shooter control
            ShooterControl();
            IT.setPower(0);

            // Panels debug view
            telemetryM.debug("Shooter PIDF Active");
            telemetryM.debug("Target Velocity  : " + TargetVelo);
            telemetryM.debug("Left Velocity    : " + SL.getVelocity());
            telemetryM.debug("Right Velocity   : " + SR.getVelocity());
            telemetryM.debug("Kp: " + Kp + " Ki: " + Ki + " Kd: " + Kd + " Kf: " + Kf);
            telemetryM.debug("Integral " + ShooterController.Integral);
            telemetryM.update(telemetry);
        }
    }
}
