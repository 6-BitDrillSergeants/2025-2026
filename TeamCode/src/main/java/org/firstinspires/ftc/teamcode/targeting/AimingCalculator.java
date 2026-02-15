package org.firstinspires.ftc.teamcode.targeting;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.config.Goal;
import org.firstinspires.ftc.teamcode.config.GoalConfig;

public final class AimingCalculator {
    private static final double BLUE_GOAL_X = 0.0;
    private static final double RED_GOAL_X = 144.0;
    private static final double GOAL_Y = 144.0;

    private static final double FIELD_CENTER = 72.0;
    private static final double OFFSET_MAX = 8.0;

    private AimingCalculator() {
        // static utility
    }

    /**
     * Returns the dynamically-shifted goal point (x,y) the robot should aim toward.
     * Heading is unused and set to 0.
     */
    public static Pose computeDynamicGoalPose(Pose currentPose, Goal goal) {
        double baseGoalX = (goal == Goal.BLUE) ? BLUE_GOAL_X : RED_GOAL_X;

        double targetX = baseGoalX; //computeDynamicGoalX(currentPose.getX(), baseGoalX);
        double targetY = computeDynamicGoalY(currentPose.getY());

        return new Pose(targetX, targetY, 0.0);
    }

    /**
     * Returns a new Pose with the same x/y as the robot,
     * but a heading that aims at the dynamically-shifted goal point.
     */
    public static Pose computeAimPose(Pose currentPose, Goal goal) {
        Pose target = computeDynamicGoalPose(currentPose, goal);

        double deltaX = target.getX() - currentPose.getX();
        double deltaY = target.getY() - currentPose.getY();

        double headingRad = Math.atan2(deltaY, deltaX);

        return new Pose(
                currentPose.getX(),
                currentPose.getY(),
                headingRad
        );
    }

    public static double computeAimHeadingRad(Pose currentPose, Goal goal) {
        return computeAimPose(currentPose, goal).getHeading();
    }

    // ----------------------------
    // INTERNAL COMPUTATION
    // ----------------------------

    private static double computeDynamicGoalX(double robotX, double baseGoalX) {
        double t =
                (GoalConfig.goal == Goal.BLUE)
                        ? clamp((FIELD_CENTER - robotX) / FIELD_CENTER, 0.0, 1.0)  // 1 at x=0, 0 at x>=72
                        : clamp((robotX - FIELD_CENTER) / FIELD_CENTER, 0.0, 1.0); // 1 at x=144, 0 at x<=72

        double inwardOffset = t * OFFSET_MAX;

        return (GoalConfig.goal == Goal.BLUE)
                ? baseGoalX + inwardOffset
                : baseGoalX - inwardOffset;
    }

    private static double computeDynamicGoalY(double robotY) {

        // Only shift goal when robot is past Y = 90
        if (robotY <= 90.0) {
            return GOAL_Y;
        }

        double offsetFactor =
                clamp(robotY - FIELD_CENTER, 0.0, FIELD_CENTER)
                        / FIELD_CENTER;

        return lerp(GOAL_Y, GOAL_Y - OFFSET_MAX, offsetFactor);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static double lerp(double start, double end, double t) {
        return start + (end - start) * t;
    }
}
