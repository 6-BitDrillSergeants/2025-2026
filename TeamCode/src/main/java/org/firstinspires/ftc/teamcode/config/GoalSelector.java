package org.firstinspires.ftc.teamcode.config;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public final class GoalSelector {
    private GoalSelector() {}

    /** Call this every loop while waiting for start (or during teleop if you want). */
    public static void update(Gamepad gamepad, Servo light, TelemetryManager telemetryM) {
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
            light.setPosition(.28);

        } else {
            gamepad.setLedColor(0.0, 0.0, 1.0, Gamepad.LED_DURATION_CONTINUOUS);
            light.setPosition(.6);
        }

        telemetryM.debug("Goal select: press BACK to toggle", GoalConfig.goal);
    }
}

