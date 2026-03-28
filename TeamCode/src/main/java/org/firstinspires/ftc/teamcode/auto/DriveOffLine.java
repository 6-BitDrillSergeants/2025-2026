package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.GoalConfig;
import org.firstinspires.ftc.teamcode.config.GoalSelector;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants;
import org.firstinspires.ftc.teamcode.subsystems.PosePublisher;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Drive off line", group = "auto")
public class DriveOffLine extends NextFTCOpMode {
    public final static class AutoPaths {
        private final Pose blueStartingPose = new Pose(55, 8, Math.toRadians(90));

        private final Pose endPose = new Pose(55, 24);

        public final AutoPathSpec movePath = new AutoPathSpec()
                .addLine(blueStartingPose,
                        endPose)
                .tangentHeading();

        public Pose getStartingPose() {
            return FieldMirror.getPose(blueStartingPose, GoalConfig.goal);
        }
    }

    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    private Servo light;

    private final PedroComponent pedroComponent = new PedroComponent(AutoConstants::createFollower);


    private Follower follower;

    public DriveOffLine() {
        // Register components BEFORE init runs
        addComponents(
                pedroComponent
        );
    }

    @Override
    public void onInit() {
        super.onInit();

        follower = PedroComponent.follower();
        light = hardwareMap.get(Servo.class, "light");

        addComponents(
                new SubsystemComponent(
                        new PosePublisher(follower)
                )
        );
    }

    private Command autoRoutine() {
        AutoPaths paths = new AutoPaths();

        follower.setStartingPose(paths.getStartingPose());

        return new SequentialGroup(
                // Move forward to shoot preloaded balls
                new FollowPath(paths.movePath.build(follower, GoalConfig.goal))
        );
    }

    @Override
    public void onWaitForStart() {
        super.onWaitForStart();
        GoalSelector.update(gamepad1, light, telemetryM);
        telemetryM.update(telemetry);
    }

    @Override
    public void onStartButtonPressed() {
        autoRoutine().invoke();
    }
}
