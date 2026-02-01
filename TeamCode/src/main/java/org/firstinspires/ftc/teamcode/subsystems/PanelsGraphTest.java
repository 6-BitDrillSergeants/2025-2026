package org.firstinspires.ftc.teamcode.subsystems;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "Test Graph")
@Configurable
public class PanelsGraphTest extends OpMode {
    private TelemetryManager panelsTelemetry;
    private VoltageSensor voltageSensor;

    public static int speed = 500;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        motor = hardwareMap.get(DcMotorEx.class, "launcher");

        motor.setVelocity(500);
    }

    @Override
    public void loop() {
        motor.setVelocity(speed);
        updateSignals();
    }

    public DcMotorEx motor;

    private void updateSignals() {
        panelsTelemetry.addData("Battery voltage", voltageSensor.getVoltage());
        panelsTelemetry.addData("Speed", motor.getVelocity());
        panelsTelemetry.addData("Current", motor.getCurrent(CurrentUnit.MILLIAMPS));

        panelsTelemetry.update(telemetry);
    }
}
