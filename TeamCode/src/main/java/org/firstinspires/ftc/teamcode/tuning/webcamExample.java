package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "Webcam AprilTag Example")
public class webcamExample extends OpMode {


    webcam aprilTagWebcam = new webcam();

    @Override
    public void init() {

        aprilTagWebcam.init(hardwareMap, telemetry);
        telemetry.addLine("Initializing webcam...");
        telemetry.update();
    }

    @Override
    public void loop() {


        aprilTagWebcam.update();


        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(24);
        aprilTagWebcam.displayDetectionTelemetry(id20);
        telemetry.addLine("=== AprilTag Detection ===");

        if (id20 != null) {
            telemetry.addLine("✔ Found AprilTag ID 20");
        }
        else
        {
            telemetry.addLine("❌ ID 20 not found");
        }

        telemetry.update();
    }
}
