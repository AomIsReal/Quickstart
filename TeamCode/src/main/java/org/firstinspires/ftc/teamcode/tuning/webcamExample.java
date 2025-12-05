package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "Webcam AprilTag – Detect All Tags")
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
        List<AprilTagDetection> allTags = aprilTagWebcam.getAllDetections();
        telemetry.addLine("=== AprilTag Detected List ===");
        if (allTags.isEmpty())
        {
            telemetry.addLine("❌ No AprilTag detected");
        }
        else
        {
            telemetry.addData("Total Tags", allTags.size());


            for (AprilTagDetection tag : allTags) {
                telemetry.addLine("-----------------------");
                aprilTagWebcam.displayDetectionTelemetry(tag);
            }
        }

        telemetry.update();
    }


}
