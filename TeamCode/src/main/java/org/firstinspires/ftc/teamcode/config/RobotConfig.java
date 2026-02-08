package org.firstinspires.ftc.teamcode.config;

import com.pedropathing.geometry.Pose;

public final class RobotConfig {
    public final static String leftFrontMotorName = "leftFront";
    public final static String rightFrontMotorName = "rightFront";
    public final static String leftBackMotorName = "leftBack";
    public final static String rightBackMotorName = "rightBack";
    public final static String flywheelMotorName = "flywheel";
    public final static String intakeMotorName = "intake";
    public final static String kickstandMotorName = "kickstand";
    public final static String paddleServoName = "paddle";
    public final static String pinpointName = "pinpoint";

    private static Pose currentPose = null;

    public static Pose getStartingPose(boolean reset) {
        if (currentPose == null || reset) {
            if (GoalConfig.goal == Goal.BLUE) {
                return new Pose(56, 12, Math.toRadians(90));
            } else {
                return new Pose(87, 12, Math.toRadians(90));
            }
        } else {
            return currentPose;
        }
    }

    public static void setCurrentPose(Pose newPose) {
        currentPose = newPose;
    }

    private RobotConfig() {
    }
}
