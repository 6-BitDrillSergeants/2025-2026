package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp
public class Drive extends OpMode {
    private DriveController driveController;
    private LauncherController launcher;
    private PaddleController paddle;

    DcMotorEx motor1;// = hardwareMap.get(DcMotorEx .class, "motor1");
    DcMotorEx motor2;// = hardwareMap.get(DcMotorEx.class, "motor2");

    Servo light;

    private int loopCount = 0;

    private boolean reverse = true;
   
    @Override
    public void init() {
        driveController = new DriveController(hardwareMap, telemetry);
        paddle = new PaddleController(hardwareMap);
        launcher = new LauncherController(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motor1 = hardwareMap.get(DcMotorEx .class, "motor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        light = hardwareMap.get(Servo.class, "light");
    }

    @Override
    public void stop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running");

        // Inverting controller inputs for cup 1
        if (reverse) {
            light.setPosition(.45); //green
            driveController.updateDriveInput(gamepad1.left_bumper, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x * -1);
        }else{
            light.setPosition(.3);//red
            driveController.updateDriveInput(gamepad1.left_bumper, gamepad1.left_stick_y * -1, gamepad1.right_stick_x, gamepad1.left_stick_x);
        }


        if (gamepad1.aWasPressed()) {
            launcher.runFast();
        } 
        else if (gamepad1.xWasPressed()) {
            launcher.runSlow();
        } 
        else if (gamepad1.bWasPressed()) {
            launcher.stop();
        }
        else if (gamepad1.yWasPressed()) {
            paddle.raisePaddle();
            loopCount = 0;
        }

        if (gamepad1.rightBumperWasPressed()){
            reverse = !reverse;
        }

        if (paddle.isRaised()) {
            loopCount++;
            if (loopCount >= 25) {
                paddle.lowerPaddle();
            }
        }

        telemetry.addData("Motor 1 Power:", motor1.getPower());
        telemetry.addData("Motor 2 Power:", motor2.getPower());
        telemetry.addData("Motor 1 Speed:", motor1.getVelocity());
        telemetry.addData("Motor 2 Speed:", motor2.getVelocity());
        
        telemetry.update();
    }
}