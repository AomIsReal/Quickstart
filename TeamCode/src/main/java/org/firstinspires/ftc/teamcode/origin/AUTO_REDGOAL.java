package org.firstinspires.ftc.teamcode.origin;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "AUTO_REDGOAL", group = "Autonomous", preselectTeleOp = "Tele")
@Configurable // Panels
public class AUTO_REDGOAL extends Robot {

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

    public PathChain Shoot1, GoPPG,collectGPP, ShootGPP, Shoot2, collectPGP, openGate, ShootPGP, GoPGP, collectPPG, ShootPPG, GoHuman, collectHuman, Shoot3, freeSpace, GoGPP, Shoot4;

    public void BuildPaths() {
        Shoot1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(111.000, 135.000), new Pose(107.000, 107.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(46))
                .build();

        GoPPG = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(107.000, 107.000), new Pose(100.375, 83.003))
                )
                .setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(0))
                .build();

        collectPPG = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(100.375, 83.003), new Pose(126.000, 83.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        openGate = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(126.000, 83.000), new Pose(127.000, 75.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Shoot2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(127.000, 75.000), new Pose(107.000, 107.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(46))
                .build();

        GoPGP = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(107.000, 107.000), new Pose(100.000, 59.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(0))
                .build();

        collectPGP = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(100.000, 59.000), new Pose(127.000, 59.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Shoot3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(127.000, 59.000), new Pose(107.000, 107.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(46))
                .build();

        GoGPP = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(107.000, 107.000), new Pose(100.000, 35.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        collectGPP = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(100.000, 35.000), new Pose(127.000, 35.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        Shoot4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(127.000, 35.000), new Pose(107.000, 107.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(46))
                .build();

        freeSpace = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(107.000, 107.000), new Pose(116.590, 97.094))
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(0)
                )
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                TargetVelo = 1200;
                Dual_SHMotor();
                follower.followPath(Shoot1,1.0, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 7.0) {
                    WaitForVelo(1150, 3, 1, null, true);
                    InTakeIN();
                    follower.followPath(GoPPG,0.8,  true);
                    setPathState(2);
                }
            case 2:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    follower.followPath(collectPPG,0.8,  true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    follower.followPath(openGate, 1.0, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    follower.followPath(Shoot2, 1.0, true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 7.0) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    IT.setPower(0);
                    WaitForVelo(1150, 3, 1, null, true);
                    InTakeIN();
                    follower.followPath(GoPGP, 0.8,  true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(collectPGP, 1.0,true);

                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 7.0) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(Shoot3, true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    IT.setPower(0);
                    WaitForVelo(1150, 3, 1, null, true);
                    InTakeIN();
                    follower.followPath(GoGPP,1.0,  true);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    follower.followPath(collectGPP,0.8,  true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    follower.followPath(Shoot4, 1.0, true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 7.0) {
                    IT.setPower(0);
                    WaitForVelo(1150, 3, 1, null, true);
                    follower.followPath(freeSpace, 1.0, true);
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
            HoodPos = 0.5;
            SetServoPos(HoodPos, AG);
            while (opModeIsActive()) {
                if(System.nanoTime() * 1E-9 - startTime > 24.5) setPathState(11);

//                if (System.nanoTime() * 1E-9 - StartPathTime > 7.0) setPathState();
                follower.update();
                autonomousPathUpdate();
                AutoAim();

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


