package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.HardwareNames.backL;
import static org.firstinspires.ftc.teamcode.HardwareNames.backR;
import static org.firstinspires.ftc.teamcode.HardwareNames.frontL;
import static org.firstinspires.ftc.teamcode.HardwareNames.frontR;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
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

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.6078)
            .forwardZeroPowerAcceleration(-29.9662)
            .lateralZeroPowerAcceleration(-67.0801)
            .automaticHoldEnd(true)

            .headingPIDFCoefficients(new PIDFCoefficients(0.85 ,0,0.1,0.01))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.0,0.0,0.0))
            .centripetalScaling(0.0);



    public static MecanumConstants driveConstants = new MecanumConstants()
            //motor
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            //Drive constant
            .maxPower(1)
            .useVoltageCompensation(true)
            .nominalVoltage(13)

            .useBrakeModeInTeleOp(true)

            .xVelocity(74.2403)
            .yVelocity(58.7281);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(4.75)
            .strafePodX(4.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


    public static PathConstraints pathConstraints = new PathConstraints(0.95, 50, 4, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();

    }


}