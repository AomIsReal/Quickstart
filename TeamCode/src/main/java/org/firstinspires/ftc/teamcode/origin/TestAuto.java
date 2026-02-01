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

@Autonomous(name = "TestAuto", group = "Autonomous")
@Configurable // Panels
public class TestAuto extends Robot {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Timer pathTimer, actionTimer, opmodeTimer;

    private void Init() {
        // Initialize Robot
        Initialize();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(0)));
        BuildPaths();
        pathTimer = new Timer();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }


    public PathChain path1, path2, path3;

    public void BuildPaths() {
        path1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(0, 0), new Pose(24, 0))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(24, 0), new Pose(24, -24)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(24, -24), new Pose(0, 0)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path1, true);
                TargetVelo = 1500;
                Dual_SHMotor();

                setPathState(1);
                break;
//            case 1:
//                if (!follower.isBusy()) {
//                    follower.followPath(path2, true);
//                    setPathState(2);
//                    sleep(1000);
//                }
//                break;
//            case 2:
//                if (!follower.isBusy()) {
//                    follower.followPath(path3, true);
//                    setPathState(-1);
//                    sleep(1000);
//                }
//                break;
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


