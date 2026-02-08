package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.RobotConfig;


@Autonomous(name = "Set paddle server to .5", group = "setup")
public final class PointFiveServo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo clawServo = hardwareMap.get(Servo.class, RobotConfig.paddleServoName);
        final double servoPos = 0.5;
        waitForStart();
        // IMPORTANT: Ensure gears are not connected when running this program.
        clawServo.setPosition(servoPos);

        telemetry.addData("Status", "Servo set to .5 position");
        telemetry.update();

        sleep(5000);
    }
}