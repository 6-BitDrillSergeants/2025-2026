package org.firstinspires.ftc.teamcode.auto;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.GoalConfig;
import org.firstinspires.ftc.teamcode.config.GoalSelector;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Paddle;
import org.firstinspires.ftc.teamcode.subsystems.PosePublisher;
import org.firstinspires.ftc.teamcode.subsystems.commands.WaitUntilCommand;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Shoot 12 balls", group = "auto")
public class TwelveBallAuto extends NextFTCOpMode {
    public final class AutoPaths {
        private final Pose blueStartingPose = new Pose(55, 8, Math.toRadians(90));
        //private final Pose farShootingPose = new Pose(55, 10, Math.toRadians(100));
        private final Pose shortShootingPose = new Pose(55, 88, Math.toRadians(135));

        private final Pose endPose = new Pose(44, 122);

        public final AutoPathSpec shootPreloadPath = new AutoPathSpec()
                .addLine(blueStartingPose,
                        shortShootingPose)
                .linearHeading(Math.toRadians(90), Math.toRadians(135));

        public final AutoPathSpec collect4_6 = new AutoPathSpec()
                .addCurve(
                        shortShootingPose,
                        new Pose(71, 31),
                        new Pose(24, 33)
                )
                .tangentHeading();

        public final AutoPathSpec shoot4_6 = new AutoPathSpec()
                .addCurve(
                        new Pose(24, 33),
                        new Pose(71, 31),
                        shortShootingPose
                )
                .linearHeading(Math.toRadians(180), Math.toRadians(135));

        public final AutoPathSpec collect7_9a = new AutoPathSpec()
                .addLine(shortShootingPose,
                        new Pose(47, 60))
                .linearHeading(Math.toRadians(135), Math.toRadians(180));

        public final AutoPathSpec collect7_9b = new AutoPathSpec()
                .addLine(new Pose(47, 60),
                        new Pose(24, 58))
                .tangentHeading();

        public final AutoPathSpec shoot7_9 = new AutoPathSpec()
                .addCurve(new Pose(24, 58),
                        new Pose(71, 31),
                        shortShootingPose)
                .linearHeading(Math.toRadians(180), Math.toRadians(135));

        public final AutoPathSpec collect10_12 = new AutoPathSpec()
                .addLine(shortShootingPose,
                        new Pose(24, 80))
                .constantHeading(Math.toRadians(180));

        public final AutoPathSpec shoot10_12 = new AutoPathSpec()
                .addLine(new Pose(19, 80),
                        shortShootingPose)
                .linearHeading(Math.toRadians(180), Math.toRadians(135));

        public final AutoPathSpec moveOffLine = new AutoPathSpec()
                .addLine(shortShootingPose,
                        endPose)
                .constantHeading(135);

        public Pose getStartingPose() {
            return FieldMirror.getPose(blueStartingPose, GoalConfig.goal);
        }
    }

    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    public TwelveBallAuto() {
        addComponents(
                new SubsystemComponent(
                        Flywheel.INSTANCE,
                        Paddle.INSTANCE,
                        Intake.INSTANCE,
                        PosePublisher.INSTANCE
                ),
                new PedroComponent(Constants::createFollower)
        );
    }

    private Command autoRoutine() {
        AutoPaths paths = new AutoPaths();

        follower().setStartingPose(paths.getStartingPose());

        return new SequentialGroup(
                Paddle.INSTANCE.lower,

                // Enable distance-based flywheel RPM
                new InstantCommand(Flywheel.INSTANCE::enableAutoFromDistance),

                // Move forward to shoot preloaded balls
                new FollowPath(paths.shootPreloadPath.build(follower(), GoalConfig.goal)),

                // Shoot preloaded balls
                shootCommand(),
                new Delay(0.5),
                new InstantCommand(Intake.INSTANCE::on),
                shootCommand(),
                new Delay(0.5),
                shootCommand(),

                // Collect balls 4-6 and drive to shooting position
                new FollowPath(paths.collect4_6.build(follower(), GoalConfig.goal)),
                new InstantCommand(Intake.INSTANCE::off),

                // Drive to shooting location
                new FollowPath(paths.shoot4_6.build(follower(), GoalConfig.goal)),

                // Shoot balls 3-6
                shootCommand(),
                new Delay(0.5),
                new InstantCommand(Intake.INSTANCE::on),
                shootCommand(),
                new Delay(0.5),
                shootCommand(),

                //Collect balls 7-9
                new FollowPath(paths.collect7_9a.build(follower(), GoalConfig.goal)),
                new FollowPath(paths.collect7_9b.build(follower(), GoalConfig.goal)),
                new InstantCommand(Intake.INSTANCE::off),

                // Drive to shooting location
                new FollowPath(paths.shoot7_9.build(follower(), GoalConfig.goal)),

                //Shoot balls 7-9
                shootCommand(),
                new Delay(0.5),
                new InstantCommand(Intake.INSTANCE::on),
                shootCommand(),
                new Delay(0.5),
                shootCommand(),

                //Collect balls 10-12
                new FollowPath(paths.collect10_12.build(follower(), GoalConfig.goal)),
                new InstantCommand(Intake.INSTANCE::off),

                // Drive to shooting location
                new FollowPath(paths.shoot10_12.build(follower(), GoalConfig.goal)),

                // shoot balls 10-12
                shootCommand(),
                new Delay(0.5),
                new InstantCommand(Intake.INSTANCE::on),
                shootCommand(),
                new Delay(0.5),
                shootCommand(),

                // Stop flywheel after shooting
                new InstantCommand(Flywheel.INSTANCE::stop),
                new InstantCommand(Intake.INSTANCE::off),

                // Move off the line
                new FollowPath(paths.moveOffLine.build(follower(), GoalConfig.goal))
        );
    }

    private SequentialGroup shootCommand() {
        return new SequentialGroup(
                new WaitUntilCommand(Flywheel.INSTANCE::isAtSpeed),
                Paddle.INSTANCE.feedOnce());
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
