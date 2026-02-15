package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.8)
            .forwardZeroPowerAcceleration(-42)
            .lateralZeroPowerAcceleration(-54.6)
            .useSecondaryDrivePIDF(true)
            .useSecondaryHeadingPIDF(true)
            .holdPointHeadingScaling(1)
            .holdPointTranslationalScaling(1)
            .useSecondaryTranslationalPIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.2,0,0, 0.3))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.03,0,0,0.005))
            .headingPIDFCoefficients(new PIDFCoefficients(1,0,0,0.25))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1,0,0,0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01,0,0,0.5,0.3))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.002,0,0,0.5,0.2));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(RobotConfig.rightFrontMotorName)
            .leftFrontMotorName(RobotConfig.leftFrontMotorName)
            .rightRearMotorName(RobotConfig.rightBackMotorName)
            .leftRearMotorName(RobotConfig.leftBackMotorName)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeModeInTeleOp(true)
            .xVelocity(58)
            .yVelocity(46);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-4.7)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName(RobotConfig.pinpointName)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
