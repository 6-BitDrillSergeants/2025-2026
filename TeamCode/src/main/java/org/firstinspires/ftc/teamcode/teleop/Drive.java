package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pidTuning.LauncherFlywheelController;
import org.firstinspires.ftc.teamcode.pidTuning.LauncherFlywheelTuning;
import org.firstinspires.ftc.teamcode.targeting.AimingCalculator;

@Configurable
@TeleOp
public class Drive extends OpMode {
    private DriveController driveController;
    private LauncherController launcher;
    private PaddleController paddle;

    DcMotorEx intakeMotor;

    DcMotor flywheelMotor;
    Servo paddleServo;
    LauncherFlywheelController controller;

    DcMotorEx kickstandMotor;


    private int loopCount = 0;

    public Follower follower;

    @Override
    public void init() {
        driveController = new DriveController(hardwareMap, telemetry);
        paddle = new PaddleController(hardwareMap);
        //launcher = new LauncherController(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        //motor2 = hardwareMap.get(DcMotorEx.class, "motor2");

        paddleServo = hardwareMap.get(Servo.class, "paddleServo");
        paddleServo.setPosition(.35);
        //light = hardwareMap.get(Servo.class, "light");

        kickstandMotor = hardwareMap.get(DcMotorEx.class, "kickstandMotor");
        kickstandMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        kickstandMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap);
//        follower.deactivateAllPIDFs();
//        follower.activateTranslational();
//        follower.activateHeading();

        //launcher.runFast();

//        follower.setStartingPose(new Pose(56, 8, Math.toRadians(180)));

        DcMotorEx flywheelMotor = hardwareMap.get(DcMotorEx.class, "launcher");

        // We are doing our own velocity control -> avoid built-in velocity mode.
        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

         controller = new LauncherFlywheelController(flywheelMotor);
    }

    @Override
    public void stop() {

    }

    @Override
    public void start() {
        //intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LauncherFlywheelTuning.targetVelocity = 90;
    }

    private boolean isHolding = false;
    private boolean hasMoved = false;

    private double min1Speed = 5000;
    private double min2Speed = 5000;
    private Pose holdPosition;

    @Override
    public void loop() {
        controller.update();

        //follower.update();
        telemetry.addData("Status", "Running");
/*
        if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0) {
            driveController.updateDriveInput(false,0,0,0);

            if (!isHolding && hasMoved) {
                telemetry.addData("holding", true);
                //follower.holdPoint(follower.getPose());
                isHolding = true;
            }
        } else {
            isHolding = false;
            hasMoved = true;
            //follower.startTeleOpDrive();

            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            //follower.setTeleOpDrive(forward, strafe, turn, true);
            driveController.updateDriveInput(gamepad1.left_bumper, gamepad1.left_stick_y * -1, gamepad1.right_stick_x, gamepad1.left_stick_x);
        }*/

        driveController.updateDriveInput(gamepad1.left_bumper, gamepad1.left_stick_y * -1, gamepad1.right_stick_x, gamepad1.left_stick_x);


        if (gamepad1.rightBumperWasPressed()) {
           // Pose aimPose = AimingCalculator.computeAimPose(follower.getPose(), AimingCalculator.Goal.BLUE_GOAL);

           // follower.holdPoint(aimPose);
           // isHolding = true
            //;

            paddleServo.setPosition(.6);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            paddleServo.setPosition(.35);
        }

        if (gamepad1.aWasPressed()){
            intakeMotor.setPower(1);
        } else if (gamepad1.bWasPressed()){
            intakeMotor.setPower(0);
        }

        if (gamepad1.leftBumperWasPressed()){
            kickstandMotor.setTargetPosition(300);
            kickstandMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //kickstandMotor.setPower(15);
        }

        telemetry.addData("kickstandPosition", kickstandMotor.getCurrentPosition());


        telemetry.update();
    }
}