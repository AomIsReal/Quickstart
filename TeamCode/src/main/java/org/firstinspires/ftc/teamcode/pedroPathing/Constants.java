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
            .mass(5.5)
            .forwardZeroPowerAcceleration(-35.62513223157471)
            .lateralZeroPowerAcceleration(-44.91665550466838)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.057, 0.00, 0.014, 0.024))
            .headingPIDFCoefficients(new PIDFCoefficients(0.6, 0.0, 0.002, 0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025,0,0.00001,0.6,0.01))
            .centripetalScaling(0.0005)
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
            .xVelocity(57.04000915600333)
            .yVelocity(39.94684955104993);



    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(0.00296830299772)
            .strafeTicksToInches(0.00296830299772)
            .turnTicksToInches(0.00296830299772)
            .leftPodY(4)
            .rightPodY(-4)
            .strafePodX(-3.5)
            .leftEncoder_HardwareMapName("BR")
            .rightEncoder_HardwareMapName("BL")
            .strafeEncoder_HardwareMapName("FR")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.REVERSE)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP));


    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            1.4,
            1);




    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelIMULocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}