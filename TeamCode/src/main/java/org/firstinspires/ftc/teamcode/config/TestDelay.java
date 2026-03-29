package org.firstinspires.ftc.teamcode.config;

import android.content.Context;
import android.content.SharedPreferences;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.Gamepad;

public final class TestDelay {

    private TestDelay() {}

    // Persistent delay value
    private static int delaySeconds = 0;

    // SharedPreferences keys
    private static final String PREF_NAME = "TestDelayPrefs";
    private static final String KEY_DELAY = "delaySeconds";

    /** Load saved value (call once in init) */
    public static void load(Context context) {
        SharedPreferences prefs = context.getSharedPreferences(PREF_NAME, Context.MODE_PRIVATE);
        delaySeconds = prefs.getInt(KEY_DELAY, 0);
    }

    /** Save current value */
    private static void save(Context context) {
        SharedPreferences prefs = context.getSharedPreferences(PREF_NAME, Context.MODE_PRIVATE);
        prefs.edit().putInt(KEY_DELAY, delaySeconds).apply();
    }

    /** Call this every loop */
    public static void update(Gamepad gamepad, TelemetryManager telemetryM, Context context) {

        // Add 1 second
        if (gamepad.xWasPressed()) {
            delaySeconds++;
            save(context);
        }

        // Subtract 1 second (no negatives)
        if (gamepad.yWasPressed()) {
            //delaySeconds = Math.max(0, delaySeconds - 1);
            delaySeconds--;
            save(context);
        }

        // Telemetry (always updates)
        telemetryM.addData("Delay (seconds)", delaySeconds);
        telemetryM.addData("Delay (ms)", delaySeconds * 1000);
    }

    /** Getter for use in autonomous/teleop */
//    public static int getDelaySeconds() {
//    }
}