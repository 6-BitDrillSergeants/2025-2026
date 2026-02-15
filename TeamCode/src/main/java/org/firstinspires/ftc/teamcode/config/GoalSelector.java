package org.firstinspires.ftc.teamcode.config;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.Gamepad;

public final class GoalSelector {
    private GoalSelector() {}

    /** Call this every loop while waiting for start (or during teleop if you want). */
    public static void update(Gamepad gamepad, TelemetryManager telemetryM) {
        // Toggle goal
        if (gamepad.backWasPressed()) {
            GoalConfig.goal =
                    (GoalConfig.goal == Goal.BLUE)
                            ? Goal.RED
                            : Goal.BLUE;
        }

        // LED feedback
        if (GoalConfig.goal == Goal.RED) {
            gamepad.setLedColor(1.0, 0.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad.setLedColor(0.0, 0.0, 1.0, Gamepad.LED_DURATION_CONTINUOUS);
        }

        telemetryM.debug("Goal select: press BACK to toggle", GoalConfig.goal);
    }
}

