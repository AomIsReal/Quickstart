package org.firstinspires.ftc.teamcode.origin;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "AUTO_NB", group = "Autonomous", preselectTeleOp = "Tele")
@Configurable // Panels
public class AUTO_NB extends Robot {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Timer pathTimer, actionTimer, opmodeTimer;
    boolean Sho = false;
    public MecanumConstants drive;

    private double StartPathTime = 0;


    public boolean Busy = false;

    private void Init() {
        // Initialize Robot
        Initialize();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(111.957, 135, Math.toRadians(90)));
        BuildPaths();
        pathTimer = new Timer();

        SetServoPos(0.5, TL, TR);
        SetServoPos(0.05, AG);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    public PathChain Brake, Park, ParkRotage;

    public void BuildPaths() {
        Brake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(111.957, 132.000), new Pose(72, 132.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        Park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(72, 132.000), new Pose(72.000, 60.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();
//
        ParkRotage = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(72.000, 60.000), new Pose(72.000, 60.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Brake,1.0, true);
//                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(Park,1.0,  true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(ParkRotage,1.0,  true);
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        StartPathTime = System.nanoTime() * 1E-9;
    }

    public void runOpMode() {
        Init();
        waitForStart();
        if (opModeIsActive()) {
            double startTime = System.nanoTime() * 1E-9;
            StartPathTime = startTime;
            setPathState(0);
//            HoodPos = 0.5;
//            SetServoPos(HoodPos, AG);
            while (opModeIsActive()) {
                if(System.nanoTime() * 1E-9 - startTime > 25.5) setPathState(1);
                follower.update();
                autonomousPathUpdate();
//                AutoAim();

                // Feedback to Driver Hub for debugging
                telemetry.addData("path state", pathState);
                telemetry.addData("x", follower.getPose().getX());
                telemetry.addData("y", follower.getPose().getY());
                telemetry.addData("heading", follower.getPose().getHeading());
                telemetry.update();


            }
        }
    }
}


