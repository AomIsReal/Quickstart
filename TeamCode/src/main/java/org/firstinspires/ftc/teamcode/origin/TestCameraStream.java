package org.firstinspires.ftc.teamcode.origin;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.concurrent.atomic.AtomicReference;

import com.bylazar.camerastream.PanelsCameraStream;

@TeleOp(name = "Test Camera Stream", group = "Dev")
public class TestCameraStream extends OpMode {

    public static class Processor implements VisionProcessor, CameraStreamSource {

        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            if (width <= 0) width = 320;
            if (height <= 0) height = 240;

            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap bmp = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, bmp);
            lastFrame.set(bmp);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas,
                                int onscreenWidth,
                                int onscreenHeight,
                                float scaleBmpPxToCanvasPx,
                                float scaleCanvasDensity,
                                Object userContext) {
            // no-op
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            Bitmap bmp = lastFrame.get();
            continuation.dispatch(consumer -> consumer.accept(bmp));
        }
    }

    private final Processor processor = new Processor();
    private VisionPortal portal;
    private boolean started = false;

    @Override
    public void init() {
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "cam1"))
                .addProcessor(processor)
                .build();
    }

    @Override
    public void init_loop() {
        if (!started && portal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            PanelsCameraStream.INSTANCE.startStream(processor, 10);
            telemetry.addLine("Panels Stream Started");
            started = true;
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        // do nothing
    }

    @Override
    public void stop() {
        PanelsCameraStream.INSTANCE.stopStream();
        portal.close();
    }
}
