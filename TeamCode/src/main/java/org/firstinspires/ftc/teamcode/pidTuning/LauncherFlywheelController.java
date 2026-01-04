package org.firstinspires.ftc.teamcode.pidTuning;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import kotlin.Unit;

import static dev.nextftc.control.builder.ControlSystemBuilderKt.controlSystem;
import static java.lang.Math.abs;

public final class LauncherFlywheelController {

    private final DcMotorEx motor;
    private ControlSystem controller;

    public LauncherFlywheelController(DcMotorEx motor) {
        this.motor = motor;
        rebuildController();
    }

    private void rebuildController() {
        controller = controlSystem(system -> {
            system.velPid(LauncherFlywheelTuning.pid);
            system.basicFF(
                    LauncherFlywheelTuning.ffV,
                    LauncherFlywheelTuning.ffA,
                    LauncherFlywheelTuning.ffS
            );
            return Unit.INSTANCE;
        });
    }

    public void update() {
        // For live tuning: rebuild so new gains take effect immediately
        rebuildController();

        controller.setGoal(new KineticState(0.0, LauncherFlywheelTuning.targetVelocity));
        double power = controller.calculate(new KineticState(motor.getCurrentPosition(), motor.getVelocity()));

        motor.setPower(power);
    }

    public boolean isAtSpeed() {
        return abs(LauncherFlywheelTuning.targetVelocity - motor.getVelocity())
                <= LauncherFlywheelTuning.toleranceVelocity;
    }

    public void stop() {
        LauncherFlywheelTuning.targetVelocity = 0.0;
        motor.setPower(0.0);
    }
}
