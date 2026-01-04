package org.firstinspires.ftc.teamcode.auto;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.LauncherOld;
import org.firstinspires.ftc.teamcode.teleop.LauncherController;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Shoot 3 balls - Blue")
public class Shoot3Blue extends NextFTCOpMode {

    public Shoot3Blue() {
        // Register components: our subsystem + Pedro’s follower
        addComponents(
                new SubsystemComponent(LauncherOld.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );

    }

    // Build a simple straight path to a shooting location
    private PathChain buildPath() {
        return follower().pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 8.000), new Pose(56.000, 102))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();
    }

    // Full autonomous routine
    private Command autoRoutine() {
        follower().setStartingPose(new Pose(56, 8, Math.toRadians(180)));

        PathChain toShoot = buildPath();
        LauncherController launcher = new LauncherController(hardwareMap, telemetry);

        launcher.runSlow();

        return new SequentialGroup(

                // Drive to the spot and wait until finished following
                new FollowPath(toShoot),
                // Spin up + feed two shots with a small cadence, then stop
                new SequentialGroup(
                        LauncherOld.INSTANCE.feedOne,
                        new Delay(1),
                        LauncherOld.INSTANCE.feedOne,
                        new Delay(1),
                        LauncherOld.INSTANCE.feedOne,
                        new Delay(1),
                        new InstantCommand(launcher::stop)
                )
        );
    }


    @Override
    public void onStartButtonPressed() {
        autoRoutine().invoke(); // schedule the routine
    }
}


