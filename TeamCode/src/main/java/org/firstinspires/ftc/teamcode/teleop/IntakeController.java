package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;


@Configurable
public class IntakeController {
    private DcMotorEx intakeMotor;

    private Telemetry telemetry;

    private final double rpm = 200;



    public IntakeController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void runVelocity() {
        run(rpm);
    }

    public void stop() {
        intakeMotor.setVelocity(0);
    }

    public double getRpmVelocity() {
        return (intakeMotor.getVelocity()*60)/145.1;
    }

    public void run(double rpm) {
        double ticksPerRev = 145.1;
        double ticksPerSecond = (rpm * ticksPerRev) / 60;

        intakeMotor.setVelocity(ticksPerSecond);
    }
}
