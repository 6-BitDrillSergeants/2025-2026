package org.firstinspires.ftc.teamcode.auto;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.GoalConfig;
import org.firstinspires.ftc.teamcode.config.GoalSelector;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants;
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

@Autonomous(name = "Shoot 3 balls", group = "auto")
public class ThreeBallAuto extends NextFTCOpMode {
    public final class AutoPaths {

        private final double shootingAngle = 165; //deg
        private final Pose blueStartingPose = new Pose(25, 127, Math.toRadians(145));
        private final Pose shortShootingPose = new Pose(55, 127, Math.toRadians(shootingAngle));

        public final AutoPathSpec shootPreloadPath = new AutoPathSpec()
                .addLine(blueStartingPose,
                        shortShootingPose)
                .linearHeading(Math.toRadians(90), Math.toRadians(shootingAngle));

        public Pose getStartingPose() {
            return FieldMirror.getPose(blueStartingPose, GoalConfig.goal);
        }
    }

    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    public ThreeBallAuto() {
        addComponents(
                new SubsystemComponent(
                        Flywheel.INSTANCE,
                        Paddle.INSTANCE,
                        Intake.INSTANCE,
                        PosePublisher.INSTANCE
                ),
                new PedroComponent(AutoConstants::createFollower)
        );
    }

    @Override
    public void onInit() {
        super.onInit();
        Paddle.INSTANCE.lower.run();
        Flywheel.INSTANCE.disableAutoFromDistance();
        Flywheel.INSTANCE.stop();
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
                new InstantCommand(Intake.INSTANCE::on),
                new Delay(0.25),
                shootCommand(),
                new Delay(0.25),
                shootCommand(),

                // Stop flywheel after shooting
                new InstantCommand(Flywheel.INSTANCE::stop),
                new InstantCommand(Intake.INSTANCE::off)
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
