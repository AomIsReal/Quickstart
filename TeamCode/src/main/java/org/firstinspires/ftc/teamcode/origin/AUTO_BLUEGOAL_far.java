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

@Autonomous(name = "AUTO_BLUEGOAL_far6ball", group = "Autonomous", preselectTeleOp = "Tele")
@Configurable // Panels
public class AUTO_BLUEGOAL_far extends Robot {

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
        follower.setStartingPose(new Pose(60, 12, Math.toRadians(90)));
        BuildPaths();
        pathTimer = new Timer();

        SetServoPos(1.0, AG);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    public PathChain Shoot1, Shoot2, Park ,Turn, collecthuman;

    public void BuildPaths() {
        Shoot1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(55.500, 8.450), new Pose(60.000, 15.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(102))
                .build();

        Turn = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.000, 12.000), new Pose(46.000, 20.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(102), Math.toRadians(180))
                .build();

        collecthuman = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(46.000, 20.000), new Pose(11.000, 20.00)))
                .setTangentHeadingInterpolation()
                .build();

        Shoot2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(11.000, 20.00), new Pose(60.000, 20.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(105), Math.toRadians(105))
                .build();

        Park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.000, 20.00), new Pose(40.000, 20.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Shoot1,1.0,  true);
                setPathState(4);
                break;
            case 1:

                if (!follower.isBusy()) {
                    WaitForVelo(1580, 4, 0.5, null, true);
                    follower.followPath(Turn,1.0,  true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    InTakeIN();
                    follower.followPath(collecthuman,0.7,  true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    IT.setPower(0);
                    follower.followPath(Shoot2,1.0,  true);
                    setPathState(5);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
//                    IT.setPower(0);
//                    WaitForVelo(1580, 2, 0.5, null, true);
//                    IT.setPower(0);
                    follower.followPath(Park,1.0,  true);
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
            double StartTime = System.nanoTime() * 1E-9;
            while (opModeIsActive()) {

//                if (pathState == 3 && StartTime - System.nanoTime() * 1E-9 > 3) follower.breakFollowing();
                if(System.nanoTime() * 1E-9 - StartTime > 25.5) setPathState(4);
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


