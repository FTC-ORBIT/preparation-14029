package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "Elvator")
public class Elvator extends OpMode {
    DcMotor elvatorMotor;

    public void init() {
        elvatorMotor = hardwareMap.get(DcMotor.class, "elvatorMotor");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {elvatorMotor.setPower(-1);}
        else if (gamepad1.b){elvatorMotor.setPower(1);}
        else {elvatorMotor.setPower(0);}
    }
}