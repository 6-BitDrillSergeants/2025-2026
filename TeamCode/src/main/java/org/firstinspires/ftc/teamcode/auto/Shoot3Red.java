package org.firstinspires.ftc.teamcode.auto;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Shoot 3 balls - Red")
public class Shoot3Red extends NextFTCOpMode {

    public Shoot3Red() {
        // Register components: our subsystem + Pedro’s follower
        addComponents(
                new SubsystemComponent(Launcher.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );

        follower().setStartingPose(new Pose(72, 8, Math.toRadians(90)));
    }

    // Build a simple straight path to a shooting location
    private PathChain buildPath() {
        return follower().pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.000, 9.000), new Pose(89.460, 97.607))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();
    }

    // Full autonomous routine
    private Command autoRoutine() {
        PathChain toShoot = buildPath();

        return new SequentialGroup(
                // Make sure follower starts from a known pose if you want
                new InstantCommand(() ->
                        follower().setStartingPose(new Pose(0, 0, 0))),

                // Drive to the spot and wait until finished following
                new FollowPath(toShoot),
                // Spin up + feed two shots with a small cadence, then stop
                new ParallelGroup(
                        Launcher.INSTANCE.spinUp,
                        new SequentialGroup(
                                Launcher.INSTANCE.feedOne,
                                new Delay(0.30),
                                Launcher.INSTANCE.feedOne,
                                Launcher.INSTANCE.feedOne,
                                Launcher.INSTANCE.turnMotorsOff
                        )
                )
        );
    }

    @Override
    public void onStartButtonPressed() {
        autoRoutine().invoke(); // schedule the routine
    }
}


