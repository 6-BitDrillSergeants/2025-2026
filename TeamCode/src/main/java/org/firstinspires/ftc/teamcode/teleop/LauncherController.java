package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LauncherController {
    //private final DcMotorEx motor1;
    //private final DcMotorEx motor2;

    private Telemetry telemetry;

    private final double fastRpm = 4700;
    private final double slowRpm = 2200;



    public LauncherController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        //motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        //motor2 = hardwareMap.get(DcMotorEx.class, "motor2");

        //motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //motor1.setDirection(DcMotorEx.Direction.REVERSE);

    }

    public void runFast() {
        run(fastRpm);
    }

    public void runSlow() {
        run(slowRpm);
    }

    public void stop() {
        //motor1.setVelocity(0);
        //motor2.setVelocity(0);
    }

    private void run(double rpm) {
        double ticksPerRev = 28;
        double ticksPerSecond = (rpm * ticksPerRev) / 60;

        //motor1.setVelocity(ticksPerSecond);
        //motor2.setVelocity(ticksPerSecond);
    }

    public double getMotorVelocity() {
       // return motor1.getVelocity();
        return 0;
    }

}
