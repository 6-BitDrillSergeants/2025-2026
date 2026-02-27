package org.firstinspires.ftc.teamcode.config;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "Select Goal", group = "config")
public class SelectGoal extends NextFTCOpMode {
    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    private Servo light;

    @Override
    public void onInit() {
        super.onInit();

        light = hardwareMap.get(Servo.class, "light");
    }

    @Override
    public void onWaitForStart() {
        super.onWaitForStart();
        GoalSelector.update(gamepad1, light, telemetryM);
        telemetryM.update(telemetry);
    }

    @Override
    public void onUpdate() {
        super.onUpdate();
        GoalSelector.update(gamepad1, light, telemetryM);
        telemetryM.update(telemetry);
    }
}
