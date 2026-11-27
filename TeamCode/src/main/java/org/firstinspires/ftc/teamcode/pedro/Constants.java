package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(1.1949213283268485);
        c.yPodOffset.set(-5.457334743710015);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.36083997667237244);
                Controller secondaryTranslationalForward = Controller.proportional(0.13332072955868898);
                Controller primaryTranslationalLateral = Controller.proportional(0.41583722553348723);
                Controller secondaryTranslationalLateral = Controller.proportional(0.15364074345931614);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.013536910751513805));
                c.brake.set(Controller.proportionalFeedforward(0.011506374138786734));

                c.headingFeedback.set(Controller.proportional(4.54609929534526));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.054808648365563234, 0.004124560523022971));

                c.linearBrakeCoefficients.set(Matrix.diag(-0.4469266616190252, 0.06034440717919899));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.016554623240204916, 0.0016666097721298589));

                c.maxAchievableForwardVelocity.set(74.72073863254845);
                c.maxAchievableStrafeVelocity.set(60.39462468531947);
                c.naturalForwardDeceleration.set(37.69775444306715);
                c.naturalStrafeDeceleration.set(73.56334017813532);
            }
    );
}