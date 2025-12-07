package org.firstinspires.ftc.teamcode.origin;

import android.util.Size;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public abstract class webcam {
    private AprilTagProcessor aprilTagProcessor;

    private VisionPortal visionPortal;

    private TelemetryManager  telemetryM;

    private List<AprilTagDetection> detectedTags = new ArrayList<>();


//    Telemetry telemetry;

    public List<AprilTagDetection> getAllDetections() {
        return aprilTagProcessor.getDetections();
    }

    public void init(HardwareMap hwMap, TelemetryManager telemetry) {

//        TelemetryManager manager = PanelsTelemetry.INSTANCE.getTelemetry();
        this.telemetryM = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                //.setDrawAxes(true)
                //.setDrawCubeProjection(true)
                //.setOutputUnits(DistanceUnit.CM , AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "cam1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        builder.enableLiveView(true);

        visionPortal = builder.build();

//        if (telemetryM != null) {
//            telemetryM.setCameraStreamSource(visionPortal);   // <-- ชี้ไปที่ VisionPortal
//            telemetryM.setCameraStreamEnabled(true);          // <-- เปิด stream
//        }
    }

    public void update() {
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public void displayDetectionTelemetry(AprilTagDetection detectedID) {
        if (detectedID == null) {
            return;
        }
        if (detectedID.metadata != null) {
            this.telemetryM.addLine(String.format("\n==== (ID %d) %s", detectedID.id, detectedID.metadata.name));
            this.telemetryM.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detectedID.ftcPose.x, detectedID.ftcPose.y, detectedID.ftcPose.z));
            this.telemetryM.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch)", detectedID.ftcPose.range, detectedID.ftcPose.bearing, detectedID.ftcPose.elevation));

        } else {
            telemetryM.addLine(String.format("\n==== (ID %d) Unknown", detectedID.id));
            telemetryM.addLine(String.format("Center %6.0f %6.0f   (pixels)", detectedID.center.x, detectedID.center.y));
        }

    }
    public AprilTagDetection getTagBySpecificId(int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }
}


