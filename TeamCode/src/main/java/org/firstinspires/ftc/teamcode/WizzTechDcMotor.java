package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class WizzTechDcMotor {

    private String name;
    private DcMotor motor;

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
}
