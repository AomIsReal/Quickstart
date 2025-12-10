package org.firstinspires.ftc.teamcode.origin;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "AUTO_REDGOAL_far6ball", group = "Autonomous", preselectTeleOp = "Tele")
@Configurable // Panels
public class AUTO_REDGOAL_far extends Robot {

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
        follower.setStartingPose(new Pose(83.4, 9, Math.toRadians(90)));
        BuildPaths();
        pathTimer = new Timer();

        SetServoPos(0.38, TL, TR);
        SetServoPos(1.0, AG);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    public PathChain Shoot1, Shoot2, Park3ball;

    public void BuildPaths() {
        Shoot1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(83.400, 9.000), new Pose(107.324, 11.582))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(85))
                .build();

        Park3ball = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(83.4000, 9.000),new Pose(75.282, 47.485))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

//
//        Shoot2 = follower
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(10.000, 9.000), new Pose(60.000, 12.000))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(105))
//                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Shoot1,1.0,  true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(Park3ball,1.0,  true);
                    setPathState(-1);
                }
                break;
//            case 2:
//                if (!follower.isBusy()) {
//                    IT.setPower(0);
//                    follower.followPath(Shoot2, 1.0, true);
//                    setPathState(-1);
//                    WaitForVelo(1800, 3, 1, null);
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
            double StartTime = System.nanoTime() * 1E-9;

            SetServoPos(1.0, AG);
            WaitForVelo(1630, 4, 5, null, false);

            while (System.nanoTime() * 1E-9 - StartTime < 26.0)
            setPathState(1);
            while (opModeIsActive()) {

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


