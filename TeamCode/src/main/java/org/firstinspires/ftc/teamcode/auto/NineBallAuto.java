package org.firstinspires.ftc.teamcode.auto;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.GoalSelector;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Paddle;
import org.firstinspires.ftc.teamcode.subsystems.PosePublisher;
import org.firstinspires.ftc.teamcode.subsystems.WaitUntilCommand;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Shoot 9 balls", group = "auto")
public class NineBallAuto extends NextFTCOpMode {

    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


    public NineBallAuto() {
        addComponents(
                new SubsystemComponent(
                        Flywheel.INSTANCE,
                        Paddle.INSTANCE,
                        PosePublisher.INSTANCE
                ),
                new PedroComponent(Constants::createFollower)
        );
    }

    private PathChain buildPath() {
        return follower().pathBuilder()
                .addPath(new BezierLine(
                        RobotConfig.getStartingPose(true),
                        new Pose(56.000, 102.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();
    }

    private Command autoRoutine() {
        follower().setStartingPose(RobotConfig.getStartingPose(true));
        PathChain toShoot = buildPath();

        return new SequentialGroup(

                // Enable distance-based flywheel RPM
                new InstantCommand(Flywheel.INSTANCE::enableAutoFromDistance),

                // Drive to shooting position
                new FollowPath(toShoot),

                // Wait until flywheel reaches target RPM
                new WaitUntilCommand(Flywheel.INSTANCE::isAtSpeed),

                // Fire 3 shots
                new SequentialGroup(
                        Paddle.INSTANCE.feedOnce(),
                        new WaitUntilCommand(Flywheel.INSTANCE::isAtSpeed),
                        Paddle.INSTANCE.feedOnce(),
                        new WaitUntilCommand(Flywheel.INSTANCE::isAtSpeed),
                        Paddle.INSTANCE.feedOnce()
                ),

                // Stop flywheel after shooting
                new InstantCommand(Flywheel.INSTANCE::stop)
        );
    }

    @Override
    public void onWaitForStart() {
        super.onWaitForStart();
        GoalSelector.update(gamepad1, telemetryM);
        telemetryM.update(telemetry);
    }

    @Override
    public void onStartButtonPressed() {
        autoRoutine().invoke();
    }
}
