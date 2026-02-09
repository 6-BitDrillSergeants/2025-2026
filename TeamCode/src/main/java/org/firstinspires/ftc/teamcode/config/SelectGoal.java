package org.firstinspires.ftc.teamcode.config;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "Select Goal", group = "config")
public class SelectGoal extends NextFTCOpMode {
    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    @Override
    public void onWaitForStart() {
        super.onWaitForStart();
        GoalSelector.update(gamepad1, telemetryM);
        telemetryM.update(telemetry);
    }

    @Override
    public void onUpdate() {
        super.onUpdate();
        GoalSelector.update(gamepad1, telemetryM);
        telemetryM.update(telemetry);
    }
}
