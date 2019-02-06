package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@SuppressWarnings("deprecation")
public class WizzTechDcMotor {

    private String name;
    private DcMotor motor;
    private boolean brake;
    private Thread thread;

    public WizzTechDcMotor(LinearOpMode opMode, String name) {
        this.name = name;
        this.motor = opMode.hardwareMap.dcMotor.get(name);
    }

    public String getName() {
        return name;
    }

    public DcMotor getMotor() {
        return motor;
    }

    public void setBrake(final boolean b) {
        this.brake = b;
        if (b) {
            thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor.setTargetPosition(motor.getCurrentPosition());
                    motor.setPower(1);
                    while (brake) {
                    }
                    motor.setPower(0);
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            });
            thread.start();
        }

    }
}
