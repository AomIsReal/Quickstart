package org.firstinspires.ftc.teamcode.origin;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;

@Autonomous(name = "AUTO_BLUEGOAL", group = "Autonomous", preselectTeleOp = "Tele")
@Configurable // Panels
public class AUTO_BLUEGOAL extends Robot {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Timer pathTimer, actionTimer, opmodeTimer;
    boolean Sho = false;
    public MecanumConstants drive;


    public boolean Busy = false;

    private void Init() {
        // Initialize Robot
        Initialize();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(36, 135, Math.toRadians(90)));
        BuildPaths();
        pathTimer = new Timer();

        SetServoPos(0.5, TL, TR);
        SetServoPos(0.05, AG);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    public PathChain Shoot, GoGPP,collectGPP, ShootGPP, GOPGP, collectPGP, openGate, ShootPGP, GOPPG, collectPPG, ShootPPG, GoHuman, collectHuman, shoot, goforspace;

    public void BuildPaths() {
        Shoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(36, 135), new Pose(46.000, 96.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(144))
                .build();

        GoGPP = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(46.0, 96.0), new Pose(46.0,80.0))
                )
                .setLinearHeadingInterpolation(Math.toRadians(144),Math.toRadians(180))
                .build();

        collectGPP = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(46.0, 80.0), new Pose(14.000, 80.0))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();


        ShootGPP = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(14.000, 80.00), new Pose(46.000, 96.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(144))
                .build();

        GOPGP = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(46, 84.000),
                                new Pose(42, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        collectPGP = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(42.000, 60.000), new Pose(14.000, 60.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        openGate = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(14.00, 60.000), new Pose(15.000, 65.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        ShootPGP = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(19.000, 65.000), new Pose(46.000, 96.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(144))
                .build();

        GOPPG = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(62.000, 84.000),
                                new Pose(46.000, 35.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        collectPPG = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(42.000, 35.500), new Pose(14.000, 35.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        ShootPPG = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(15.000, 35.500),
                                new Pose(46.000, 96.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(144))
                .build();

        GoHuman = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(62.000, 84.000), new Pose(9.000, 47.742))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-90))
                .build();

        collectHuman = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(10.000, 47.742), new Pose(9.000, 10.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        shoot = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(9.000, 10.000),
                                new Pose(69.472, 55.191),
                                new Pose(46.000, 96.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180))
                .build();

        goforspace = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(62.310, 84.555), new Pose(19.913, 101.028))
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(-180),
                        Math.toRadians(-180)
                )
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                TargetVelo = 1200;
                Dual_SHMotor();
                follower.followPath(Shoot,1.0, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 7.0) {
                    WaitForVelo(1150, 3, 1, null, true);
                    InTakeIN();
                    follower.followPath(collectGPP,0.8,  true);
                    setPathState(14);
                }
                break;
            case 2:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    follower.followPath(ShootGPP, 1.0, true);
                    setPathState(3);

                }
                break;
            case 3:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 7.0) {
                    IT.setPower(0);
                    WaitForVelo(1150, 3, 1, null, true);
                    InTakeIN();
                    follower.followPath(GOPGP, 1.0, true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(collectPGP, 0.8,  true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(ShootPGP, 1.0,true);

                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 7.0) {
                    IT.setPower(0);
                    WaitForVelo(1150, 3, 1, null, true);
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(openGate, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    InTakeIN();
                    follower.followPath(GOPPG,1.0,  true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    follower.followPath(collectPPG,0.8,  true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    follower.followPath(ShootPPG, 1.0, true);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 7.0) {
                    IT.setPower(0);
                    WaitForVelo(1150, 3, 1, null, true);
                    follower.followPath(GoHuman, 1.0, true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    follower.followPath(collectHuman, 0.75, true);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    follower.followPath(shoot, 1.0, true);
                    setPathState(13);
                }
                break;
            case 13:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 7.0) {
                    IT.setPower(0);
                    WaitForVelo(1150, 3, 1, null, true);
                    follower.followPath(goforspace,1.0, true);
                    setPathState(-1);
                }
                break;
            case 14:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    follower.followPath(GoGPP, 1.0,  true);
                    setPathState(2);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void runOpMode() {
        Init();
        waitForStart();
        if (opModeIsActive()) {
            double startTime = System.nanoTime() * 1E-9;
            setPathState(0);
            HoodPos = 0.5;
            SetServoPos(HoodPos, AG);
            while (opModeIsActive()) {
                if(System.nanoTime() * 1E-9 - startTime > 28.0) setPathState(13);

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


