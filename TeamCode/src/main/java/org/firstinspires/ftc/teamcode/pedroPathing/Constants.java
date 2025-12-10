package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.8)
            .forwardZeroPowerAcceleration(-32.260681903807516)
            .lateralZeroPowerAcceleration(-58.601645424315514)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.03, 0.00, 0.003, 0.024))
            .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0.0, 0.002, 0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025,0,0.00005,0.6,0.008))
            .centripetalScaling(0.0005)
            .holdPointHeadingScaling(0.05)
            .holdPointTranslationalScaling(0.3)
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(80.64564484204381)
            .yVelocity(65.23230367685088);



    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(0.00296830299772)
            .strafeTicksToInches(0.00296830299772)
            .turnTicksToInches(0.00296830299772)
            .leftPodY(4.7)
            .rightPodY(-4.7)
            .strafePodX(-1.1)
            .leftEncoder_HardwareMapName("FL")
            .rightEncoder_HardwareMapName("FR")
            .strafeEncoder_HardwareMapName("BL")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP));


    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            0.95,
            1.1);




    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelIMULocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}