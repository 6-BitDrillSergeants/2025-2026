package org.firstinspires.ftc.teamcode.pidTuning;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * TeleOp for live-tuning the launcher flywheel velocity controller.
 *
 * Tuning inputs come from LauncherFlywheelTuning (Panels).
 * - targetVelocity (ticks/sec)
 * - pid (PIDCoefficients)
 * - ffV/ffA/ffS
 * - toleranceVelocity
 */
@TeleOp(name = "TUNE: Launcher Flywheel Velocity", group = "Tuning")
@Configurable
public class TuneLauncher extends LinearOpMode {

    private TelemetryManager panelsTelemetry;

    Servo paddleServo;

    @Override
    public void runOpMode() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        panelsTelemetry.addData("Target", 0);
        panelsTelemetry.addData("Error", 0);
        panelsTelemetry.addData("Measured", 0);

        panelsTelemetry.update();

        DcMotorEx flywheelMotor = hardwareMap.get(DcMotorEx.class, "launcher");

        // We are doing our own velocity control -> avoid built-in velocity mode.
        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        LauncherFlywheelController controller = new LauncherFlywheelController(flywheelMotor);

        paddleServo = hardwareMap.get(Servo.class, "paddleServo");
        paddleServo.setPosition(.58); // .35


        waitForStart();

        while (opModeIsActive()) {
            // Optional: quick driver controls while tuning
            // - A: spin at current targetVelocity (Panels)
            // - B: stop
            if (gamepad1.b) {
                controller.stop();
            }

            // Run tuning controller continuously
            controller.update();

            double measuredVel = flywheelMotor.getVelocity();
            double targetVel = LauncherFlywheelTuning.targetVelocity;
            double error = targetVel - measuredVel;

            if (gamepad1.rightBumperWasPressed()) {
                paddleServo.setPosition(.83); // .60
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                paddleServo.setPosition(.58); // .35
            }

            panelsTelemetry.addData("Target", targetVel);
            panelsTelemetry.addData("Error", error);
            panelsTelemetry.addData("Measured", measuredVel);

            panelsTelemetry.addData("RPM", convertToRpm(measuredVel));

            panelsTelemetry.addData("position", flywheelMotor.getCurrentPosition());
            panelsTelemetry.addData("amps", flywheelMotor.getCurrent(CurrentUnit.AMPS));

            //flywheelMotor.setVelocity(targetVel);
            panelsTelemetry.update();
        }

        controller.stop();
    }

    private double convertToRpm(double ticksPerSecond) {
        return (ticksPerSecond / 28) * 60;
    }
}

