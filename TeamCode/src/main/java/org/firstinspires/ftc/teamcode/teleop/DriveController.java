package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.teamcode.utilities.Easing;
//import org.firstinspires.ftc.teamcode.utilities.Shoulder_PIDF;

public class DriveController {
    private final double FAST_SPEED_SCALE = 1.0;
    private final double SLOW_SPEED_SCALE = .5;

    private final Telemetry telemetry;

    private final DcMotor motorFL;
    private final DcMotor motorBL;
    private final DcMotor motorFR;
    private final DcMotor motorBR;

    private boolean throttleButtonPressed = false;
    private boolean throttleDown = false;

    public DriveController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        motorFL = hardwareMap.get(DcMotor.class, "leftFront");
        motorFR = hardwareMap.get(DcMotor.class, "rightFront");
        motorBL = hardwareMap.get(DcMotor.class, "leftBack");
        motorBR = hardwareMap.get(DcMotor.class, "rightBack");

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void updateDriveInput(boolean throttleButtonPressed, double pitch, double yaw, double roll) {
        // toggle throttle on button release
        if (this.throttleButtonPressed && !throttleButtonPressed) {
            throttleDown = !throttleDown;
        }
        this.throttleButtonPressed = throttleButtonPressed;

        //pitch = Easing.QuadraticEase(pitch);
        //yaw = Easing.QuadraticEase(yaw);
        //roll = Easing.QuadraticEase(roll);

        adjustMotorPower(pitch, yaw, roll);
    }

    private void adjustMotorPower(double pitch, double yaw, double roll) {
        double scaledPower = throttleDown ? SLOW_SPEED_SCALE : FAST_SPEED_SCALE;

        double frontRightPower = (pitch - yaw - roll);
        double frontLeftPower = (pitch + yaw + roll);
        double backRightPower = (pitch - yaw + roll);
        double backLeftPower = (pitch + yaw - roll);

        // Scales motor power such that max power is never larger in magnitude than 1.
        double cur_max = Math.max(Math.max(Math.abs(frontRightPower), Math.abs(frontLeftPower)), Math.max(Math.abs(backRightPower), Math.abs(backLeftPower)));
        if (cur_max > 1) {
            frontRightPower /= cur_max;
            frontLeftPower /= cur_max;
            backRightPower /= cur_max;
            backLeftPower /= cur_max;
        }

        frontLeftPower = frontLeftPower * scaledPower;
        frontRightPower = frontRightPower * scaledPower;
        backLeftPower = backLeftPower * scaledPower;
        backRightPower = backRightPower * scaledPower;

        motorFL.setPower(frontLeftPower);
        motorFR.setPower(frontRightPower);
        motorBL.setPower(backLeftPower);
        motorBR.setPower(backRightPower);

        telemetry.addLine();
        telemetry.addData("Throttle Down", throttleDown);
        telemetry.addData("Front Left Power", frontLeftPower);
        telemetry.addData("Front Right Power", frontRightPower);
        telemetry.addData("Back Left Power", backLeftPower);
        telemetry.addData("Back Right Power", backRightPower);

    }
}