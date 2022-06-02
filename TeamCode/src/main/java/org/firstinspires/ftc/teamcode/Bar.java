package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

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
    double lastPos = 0;
    double lastTime = 0;
    double lastMotorsVelocity = 0;
    double currentPos = 0;
    double lastAngle = 0;
    double lastRobotVelocity = 0;
    double currentMotorsVelocity = 0;
    double currentRobotVelocity = 0;
    double sumVelocity = 0;
    public static double wheelPerimeter = 9.6 * Math.PI;
    public static double ticksPerCycle = 537.6;
    ElapsedTime time = new ElapsedTime();
    double rMotorsPower = 0.5;
    double lMotorsPower = 0.5;
    double currentAngle = 0;


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
        driveRobot();
        sumVelocity = 0;
        for (int i = 0; 10 > i; i++) {
            RobotVelocity();
            sumVelocity +=currentRobotVelocity;
        }
        sumVelocity /= 10;
      telemetry.addData("currentMotorsVelocity", sumVelocity);
      telemetry.addData("distance", currentPos);
    }
    private double RobotVelocity(){
        //if (gamepad1.a){
            //moving the motors
            int leftEncoderVal = -lb.getCurrentPosition();
            //int rightEncoderVal = -rb.getCurrentPosition();
            //double encodersVal = (rightEncoderVal + leftEncoderVal) / 2;
            currentPos = (leftEncoderVal / ticksPerCycle) * wheelPerimeter;
            double currentTime = time.milliseconds() / 1000;
            double deltaPos = currentPos - lastPos;
            double deltaTime = currentTime - lastTime;
            currentRobotVelocity = deltaTime == 0?   lastRobotVelocity :  deltaPos / deltaTime;
            //calculating the currentRobotV2elocity using if else condition in case deltaTime = o
             lastPos = currentPos;
             lastTime = currentTime;
             lastRobotVelocity = currentRobotVelocity;
        //}
        return currentRobotVelocity;
    }
    private double motorsVelocity () {
        //if (gamepad1.b) {
            int leftEncoderVal = -lb.getCurrentPosition();
            //int rightEncoderVal = -rb.getCurrentPosition();
            //double encodersVal = (rightEncoderVal + leftEncoderVal) / 2;
            currentAngle = (360 * leftEncoderVal) / ticksPerCycle;
            double currentTime = time.milliseconds() / 1000;
            double deltaAngle = currentAngle - lastAngle;
            double deltaTime = currentTime - lastTime;
            currentMotorsVelocity = deltaTime == 0? lastMotorsVelocity : deltaAngle / deltaTime;
            lastAngle = currentAngle;
            lastTime = currentTime;
        //}
        lastMotorsVelocity = currentMotorsVelocity;
        return currentMotorsVelocity;
    }
    public double motorAngle (){
        int leftEncoderVal = -lb.getCurrentPosition();
        currentAngle = (360 * leftEncoderVal) / ticksPerCycle;
        return currentAngle;
    }
    public void driveRobot() {
        float lMotorsPower = (-gamepad1.left_stick_y + gamepad1.right_trigger - gamepad1.left_trigger);
        float rMotorsPower = (-gamepad1.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger);

//        final float highestPower = Math.max( Math.abs(lMotorsPower), Math.abs(rMotorsPower) );
//        lMotorsPower/=highestPower;
//        rMotorsPower/=highestPower;
        lf.setPower(lMotorsPower);
        lb.setPower(lMotorsPower);
        rf.setPower(rMotorsPower);
        rb.setPower(rMotorsPower);
    }
}