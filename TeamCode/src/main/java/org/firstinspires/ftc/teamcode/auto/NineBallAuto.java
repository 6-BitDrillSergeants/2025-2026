package org.firstinspires.ftc.teamcode.auto;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.Goal;
import org.firstinspires.ftc.teamcode.config.GoalConfig;
import org.firstinspires.ftc.teamcode.config.GoalSelector;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Paddle;
import org.firstinspires.ftc.teamcode.subsystems.PosePublisher;
import org.firstinspires.ftc.teamcode.subsystems.commands.WaitUntilCommand;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Shoot 9 balls", group = "auto")
public class NineBallAuto extends NextFTCOpMode {
    public final class AutoPaths {
        private final Pose blueStartingPose = new Pose(55,8, Math.toRadians(90));

        private final AutoPathSpec path1 = new AutoPathSpec()
                .addLine(
                        blueStartingPose,
                        new Pose(55, 20)
                )
                .constantHeading(Math.toRadians(90));

        private final AutoPathSpec path2 = new AutoPathSpec()
                .addCurve(
                        new Pose(55, 20),
                        new Pose(54, 36),
                        new Pose(24, 35)
                )
                .tangentHeading();

        private final AutoPathSpec path3 = new AutoPathSpec()
                .addCurve(
                        new Pose(24, 35),
                        new Pose(66, 53),
                        new Pose(55, 88)
                )
                .linearHeading(Math.toRadians(180), Math.toRadians(135));

        public Pose getStartingPose() {
            return FieldMirror.getPose(blueStartingPose, GoalConfig.goal);
        }

        public PathChain buildPath1(Follower follower, Goal goal) {
            return path1.build(follower, goal);
        }

        public PathChain buildPath2(Follower follower, Goal goal) {
            return path2.build(follower, goal);
        }

        public PathChain buildPath3(Follower follower, Goal goal) {
            return path3.build(follower, goal);
        }
    }

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

    private Command autoRoutine() {
        AutoPaths paths = new AutoPaths();

        follower().setStartingPose(paths.getStartingPose());

        return new SequentialGroup(

                // Enable distance-based flywheel RPM
                new InstantCommand(Flywheel.INSTANCE::enableAutoFromDistance),

                // Collect 3 balls and drive to shooting position
                new FollowPath(paths.buildPath1(follower(), GoalConfig.goal)),
                new InstantCommand(Intake.INSTANCE::on),
                new FollowPath(paths.buildPath2(follower(), GoalConfig.goal)),
                new InstantCommand(Intake.INSTANCE::off),
                new FollowPath(paths.buildPath3(follower(), GoalConfig.goal)),

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
