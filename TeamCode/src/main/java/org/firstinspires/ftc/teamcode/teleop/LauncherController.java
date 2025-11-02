package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LauncherController {
    private final DcMotorEx motor1;
    private final DcMotorEx motor2;

    private Telemetry telemetry;

    private final double fastRpm = 225;
    private final double slowRpm = 200;

    public LauncherController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void runFast() {
        run(fastRpm);       
    }

    public void runSlow() {
        run(slowRpm);
    }

    public void stop() {
        motor1.setVelocity(0);
        motor2.setVelocity(0);
    }

    private void run(double rpm) {
        double ticksPerRev = 383.6;
        double ticksPerSecond = (rpm * ticksPerRev) / 60;

        motor1.setVelocity(ticksPerSecond);
        motor2.setVelocity(ticksPerSecond);

        telemetry.addData("Motor 1 Power:", motor1.getPower());
        telemetry.addData("Motor 2 Power:", motor2.getPower());
    }
}
