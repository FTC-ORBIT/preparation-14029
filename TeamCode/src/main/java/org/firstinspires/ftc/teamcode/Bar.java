package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "Bar's Teleop")
public class Bar extends OpMode {
    private DcMotor rf;
    private DcMotor rb;
    private DcMotor lf;
    private DcMotor lb;
    public static double wheelPerimeter = 9.6 * Math.PI;
    public static double tickesPerCycle = 537.6;
    ElapsedTime time = new ElapsedTime();


    @Override
    public void init() {
        time.reset();
        rf = hardwareMap.get(DcMotor.class, "rf");
        rb = hardwareMap.get(DcMotor.class, "rb");
        lf = hardwareMap.get(DcMotor.class, "lf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);
        rb.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);;
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //initializing motors
    }

    @Override
    public void loop() {
        float lMotorsPower = (-gamepad1.left_stick_y + gamepad1.right_trigger - gamepad1.left_trigger);
        float rMotorsPower = (-gamepad1.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger);
        lf.setPower(lMotorsPower);
        lb.setPower(lMotorsPower);
        rf.setPower(rMotorsPower);
        rb.setPower(rMotorsPower);
        //moving the motors
        double leftEncoderVal = -lb.getCurrentPosition();
        double rightEncoderVal = -rb.getCurrentPosition();
        double encodersVal = (rightEncoderVal + leftEncoderVal) / 2;
        double currentPos = (encodersVal / tickesPerCycle) * wheelPerimeter;
        telemetry.addData("distance", currentPos);
        double currentTime = time.milliseconds() / 1000;
        double velocity = currentPos / currentTime; //calculating the velocity
        telemetry.addData("velocity", velocity);
    }
}
