package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class WizzTechDcMotor {

    private String name;
    private DcMotor motor;

    public WizzTechDcMotor(String name) {
        this.name = name;
        this.motor = Robot.getInstance().getHardwareMap().dcMotor.get(name);
    }

    public String getName() {
        return name;
    }

    public DcMotor getMotor() {
        return motor;
    }
}
