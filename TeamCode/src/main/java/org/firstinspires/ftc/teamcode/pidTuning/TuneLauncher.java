package org.firstinspires.ftc.teamcode.pidTuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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
public class TuneLauncher extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotorEx flywheelMotor = hardwareMap.get(DcMotorEx.class, "launcher");

        // We are doing our own velocity control -> avoid built-in velocity mode.
        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LauncherFlywheelController controller = new LauncherFlywheelController(flywheelMotor);

        telemetry.addLine("Panels: edit LauncherFlywheelTuning.* while running");
        telemetry.addLine("Use targetVelocity in ticks/sec (same units as motor.getVelocity())");
        telemetry.update();

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

            telemetry.addData("Target (ticks/s)", targetVel);
            telemetry.addData("Measured (ticks/s)", measuredVel);
            telemetry.addData("Error (ticks/s)", error);
            telemetry.addData("At Speed", controller.isAtSpeed());

            telemetry.addData("kP", LauncherFlywheelTuning.pid.kP);
            telemetry.addData("kI", LauncherFlywheelTuning.pid.kI);
            telemetry.addData("kD", LauncherFlywheelTuning.pid.kD);
            telemetry.addData("ffV", LauncherFlywheelTuning.ffV);
            telemetry.addData("ffA", LauncherFlywheelTuning.ffA);
            telemetry.addData("ffS", LauncherFlywheelTuning.ffS);

            telemetry.update();
        }

        controller.stop();
    }
}

