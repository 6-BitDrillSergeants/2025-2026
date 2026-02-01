package org.firstinspires.ftc.teamcode.auto;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Forward 12 Inches")
public class MoveForwardAuto extends NextFTCOpMode {

    public MoveForwardAuto() {
        addComponents(
                new PedroComponent(Constants::createFollower)
        );
    }

    // Build a simple straight 12-inch forward path
    private PathChain buildForward12() {

        // Starting pose (0,0,0) → ending pose (0,12,0)
        return follower().pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(0, 0, 0),
                                new Pose(0, 12, 0)
                        )
                )
                .build();
    }

    private Command autoRoutine() {

        // Set starting pose
        follower().setStartingPose(new Pose(0, 0, 0));

        PathChain forward12 = buildForward12();

        return new SequentialGroup(
                new FollowPath(forward12)
        );
    }

    @Override
    public void onStartButtonPressed() {
        autoRoutine().invoke();
    }
}

