package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.EasyRobot;


@TeleOp(name = "TeleOp - Nationala", group = "Official")
public class TeleOpMode4 extends EasyRobot {

    boolean classic = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot(this);

        waitForStart();

        while (!gamepad1.dpad_left && !gamepad1.dpad_right && opModeIsActive()) {
            classic = gamepad1.dpad_left;
        }

        while (opModeIsActive()) {

            //driver 1
            if (classic) {
                if (gamepad1.x) {
                    getLeftMotorUp().getMotor().setPower(-0.1);
                    getRightMotorUp().getMotor().setPower(0.1);
                    getLeftMotorDown().getMotor().setPower(0.1);
                    getRightMotorDown().getMotor().setPower(-0.1);
                } else {
                    getLeftMotorUp().getMotor().setPower(Range.clip(-gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1));
                    getRightMotorUp().getMotor().setPower(Range.clip(-gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1));
                    getLeftMotorDown().getMotor().setPower(Range.clip(gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1));
                    getRightMotorDown().getMotor().setPower(Range.clip(gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1));
                }

                if (gamepad1.right_trigger > 0) {
                    getLandMotor().getMotor().setPower(-gamepad1.right_trigger);
                } else if (gamepad1.left_trigger > 0) {
                    getLandMotor().getMotor().setPower(gamepad1.left_trigger);
                } else {
                    getLandMotor().getMotor().setPower(0);
                }
            } else {
                if (gamepad1.right_trigger > 0) {
                    getLeftMotorUp().getMotor().setPower(-gamepad1.right_trigger);
                    getRightMotorUp().getMotor().setPower(gamepad1.right_trigger);
                    getLeftMotorDown().getMotor().setPower(-gamepad1.right_trigger);
                    getRightMotorDown().getMotor().setPower(gamepad1.right_trigger);
                } else if (gamepad1.left_trigger > 0) {
                    getLeftMotorUp().getMotor().setPower(gamepad1.right_trigger);
                    getRightMotorUp().getMotor().setPower(-gamepad1.right_trigger);
                    getLeftMotorDown().getMotor().setPower(gamepad1.right_trigger);
                    getRightMotorDown().getMotor().setPower(-gamepad1.right_trigger);
                }

                if (gamepad1.right_bumper) {
                    getLeftMotorUp().getMotor().setPower(-1);
                    getRightMotorUp().getMotor().setPower(-1);
                    getLeftMotorDown().getMotor().setPower(-1);
                    getRightMotorDown().getMotor().setPower(-1);
                } else if (gamepad1.left_bumper) {
                    getLeftMotorUp().getMotor().setPower(1);
                    getRightMotorUp().getMotor().setPower(1);
                    getLeftMotorDown().getMotor().setPower(1);
                    getRightMotorDown().getMotor().setPower(1);
                }

                if (gamepad1.dpad_left){
                    getLeftMotorUp().getMotor().setPower(1);
                    getRightMotorUp().getMotor().setPower(1);
                    getLeftMotorDown().getMotor().setPower(-1);
                    getRightMotorDown().getMotor().setPower(-1);
                }else if (gamepad1.dpad_right){
                    getLeftMotorUp().getMotor().setPower(-1);
                    getRightMotorUp().getMotor().setPower(-1);
                    getLeftMotorDown().getMotor().setPower(1);
                    getRightMotorDown().getMotor().setPower(1);
                }

                if (gamepad1.dpad_up) {
                    getLandMotor().getMotor().setPower(-1);
                } else if (gamepad1.dpad_down) {
                    getLandMotor().getMotor().setPower(1);
                }
            }

            //driver 2
            if (gamepad2.a) {
                getCollectorServo().setPower(1);
            } else if (gamepad2.b) {
                getCollectorServo().setPower(-1);
            }

            if (gamepad2.right_bumper) {
                if (getExtendLift().getMotor().getCurrentPosition() < 1000) { // TODO: 3/16/2019 change ticks
                    getExtendLift().getMotor().setPower(1);// TODO: 3/16/2019 change direction
                } else {
                    getExtendLift().getMotor().setPower(0);
                }
            } else if (gamepad2.left_bumper) {
                if (getExtendLift().getMotor().getCurrentPosition() > 0) {
                    getExtendLift().getMotor().setPower(-1);// TODO: 3/16/2019 change direction
                } else {
                    getExtendLift().getMotor().setPower(0);
                }
            }

            if (gamepad2.dpad_down) {
                getAxLift().setBrake(false);
                driveTicks(getAxLift(), 0, 1.0, true, false);
                getAxLift().setBrake(true);
            } else if (gamepad2.dpad_up) {
                getAxLift().setBrake(false);
                driveTicks(getAxLift(), 1000, 1.0, true, false);// TODO: 3/16/2019 pozitie 45 grade
                getAxLift().setBrake(true);
            } else if (gamepad2.y){
                getAxLift().setBrake(false);
                driveTicks(getAxLift(), 1000, 1.0, true, false);// TODO: 3/16/2019 pozitie sus
                getAxLift().setBrake(true);
            }

        }
    }
}