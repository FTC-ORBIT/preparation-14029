package org.firstinspires.ftc.teamcode;

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
    double lastPos;
    double lastTime;
    double lastAngle;
    double lastRobotVelocity;
    double currentMotorsVelocity;
    double currentRobotVelocity;
    public static double wheelPerimeter = 9.6 * Math.PI;
    public static double ticksPerCycle = 537.6;
    ElapsedTime time = new ElapsedTime();
    double rMotorsPower = 0.5;
    double lMotorsPower = 0.5;
    double currentAngle;


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
      motorsVelocity();
      telemetry.addData("currentMotorsVelocity", currentMotorsVelocity);
    }
    private double RobotVelocity(){
        if (gamepad1.a){
            lf.setPower(lMotorsPower);
            lb.setPower(lMotorsPower);
            rf.setPower(rMotorsPower);
            rb.setPower(rMotorsPower);
            //moving the motors
            int leftEncoderVal = -lb.getCurrentPosition();
            int rightEncoderVal = -rb.getCurrentPosition();
            double encodersVal = (rightEncoderVal + leftEncoderVal) / 2;
            double currentPos = (encodersVal / ticksPerCycle) * wheelPerimeter;
            telemetry.addData("distance", currentPos);
            double currentTime = time.milliseconds() / 1000;
            double deltaPos = currentPos - lastPos;
            double deltaTime = currentTime - lastTime;
            currentRobotVelocity = deltaTime == 0?   lastRobotVelocity :  deltaPos / deltaTime;
            //calculating the currentRobotVelocity using if else condition in case deltaTime = o
            telemetry.addData("currentRobotVelocity", currentRobotVelocity);
             lastPos = currentPos;
             lastTime = currentTime;
             lastRobotVelocity = currentRobotVelocity;
        }
        return currentRobotVelocity;
    }
    private double motorsVelocity () {
        //if (gamepad1.b) {
            int leftEncoderVal = -lb.getCurrentPosition();
            int rightEncoderVal = -rb.getCurrentPosition();
            double encodersVal = (rightEncoderVal + leftEncoderVal) / 2;
            currentAngle = (360 * encodersVal) / ticksPerCycle;
            double currentTime = time.milliseconds() / 1000;
            double deltaAngle = currentAngle - lastAngle;
            double deltaTime = currentTime - lastTime;
            currentMotorsVelocity = deltaAngle / deltaTime;
            lastAngle = currentAngle;
            lastTime = currentTime;
        //}
        return currentMotorsVelocity;
    }
}
