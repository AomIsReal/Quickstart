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

@Autonomous(name = "AUTO_BLUEGOAL", group = "Autonomous")
@Configurable // Panels
public class AUTO_BLUEGOAL extends Robot {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Timer pathTimer, actionTimer, opmodeTimer;

    private void Init() {
        // Initialize Robot
        Initialize();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(20.5, 122.51393948444976, Math.toRadians(144)));
        BuildPaths();
        pathTimer = new Timer();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }


    public PathChain Shoot, collectGPP, ShootGPP, GOPGP, collectPGP, openGate, ShootPGP, GOPPG, collectPPG, ShootPPG, GoHuman, collectHuman, shoot, goforspace;

    public void BuildPaths() {
        Shoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(20.500, 122.514), new Pose(62.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();

        collectGPP = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(62.000, 84.000), new Pose(15.613, 83.982))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        ShootGPP = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(15.613, 83.982), new Pose(62.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        GOPGP = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(62.000, 84.000),
                                new Pose(69.758, 70.518),
                                new Pose(42.000, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        collectPGP = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(42.000, 60.000), new Pose(15.000, 60.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        openGate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(15.000, 60.000),
                                new Pose(38.100, 70.000),
                                new Pose(15.000, 70.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();

        ShootPGP = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(15.000, 70.000), new Pose(62.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        GOPPG = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(62.000, 84.000),
                                new Pose(69.615, 65.791),
                                new Pose(42.000, 35.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        collectPPG = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(42.000, 35.500), new Pose(15.000, 35.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        ShootPPG = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(15.000, 35.500),
                                new Pose(70.000, 60.000),
                                new Pose(62.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
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
                        new BezierLine(new Pose(9.000, 47.742), new Pose(9.000, 10.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        shoot = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(9.000, 10.000),
                                new Pose(69.472, 55.191),
                                new Pose(62.310, 84.555)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180))
                .build();

        goforspace = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(62.310, 84.555), new Pose(20.913, 101.028))
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
                SetServoPos(0, TL, TR);
                follower.followPath(Shoot, true);
                ShooterControl();
                WaitForVelo();
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    InTakeIN();
                    follower.followPath(collectGPP, true);
                    setPathState(2);
                    IT.setPower(0);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(ShootGPP, true);
                    ShooterControl();
                    WaitForVelo();
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(GOPGP, true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(collectPGP, true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(ShootPGP, true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(openGate, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
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
            setPathState(0);
            while (opModeIsActive()) {

                follower.update();
                autonomousPathUpdate();
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


