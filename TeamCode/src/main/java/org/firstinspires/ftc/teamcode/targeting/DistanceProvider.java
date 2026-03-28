package org.firstinspires.ftc.teamcode.targeting;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.config.GoalConfig;

public final class DistanceProvider {

    public DistanceProvider(Follower follower) {
        this.follower = follower;
    }

    private final Follower follower;

    /**
     * Returns distance from current pose to (targetX, targetY).
     */
    public double getDistance() {
        Pose current = follower.getPose();
        Pose target = AimingCalculator.computeDynamicGoalPose(current, GoalConfig.goal);

        double dx = target.getX() - current.getX();
        double dy = target.getY() - current.getY();
        return Math.hypot(dx, dy);
    }
}
