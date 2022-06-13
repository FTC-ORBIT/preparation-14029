package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@TeleOp(name = "mecanumTest")
public class mecanum extends OpMode {
    public final DcMotor[] motors = new DcMotor[4];
    public BNO055IMU imu;

    public void init() {
        //opMode.telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        motors[0] = hardwareMap.get(DcMotor.class, "lf");
        motors[1] = hardwareMap.get(DcMotor.class, "rf");
        motors[2] = hardwareMap.get(DcMotor.class, "lb");
        motors[3] = hardwareMap.get(DcMotor.class, "rb");
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        for (final DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }


    }

    @Override

    public void loop() {
        final Vector joystick = new Vector(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
        //motortest();
        fieldCentric(joystick, gamepad1.right_trigger - gamepad1.left_trigger);
        telemetry.addData("angle", getAngle());
    }


    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void fieldCentric(final Vector joystickVector, final double r) {
        final float robotAngle = (float) Math.toRadians(getAngle());
        final Vector rotated = joystickVector.rotate(robotAngle);
        drive(rotated, r);
    }

    public void motortest() {
        if (gamepad1.a) {
            motors[3].setPower(0.5);
        } else if (gamepad1.b) {
            motors[2].setPower(0.5);
        } else if (gamepad1.x) {
            motors[0].setPower(0.5);
        } else if (gamepad1.y) {
            motors[1].setPower(0.5);
        }
    }

    public void drive(Vector drive, double r) {
        final double lfPower = drive.y - drive.x + r;
        final double rfPower = drive.y + drive.x - r;
        final double lbPower = drive.y + drive.x + r;
        final double rbPower = drive.y - drive.x - r;
        double highestPower = 1;
        final double max = Math.max(Math.abs(lfPower), Math.max(Math.abs(lbPower), Math.max(Math.abs(rfPower), Math.abs(rbPower))));
        if (max > 1) highestPower = max;
        motors[0].setPower((lfPower / highestPower));
        motors[1].setPower((rfPower / highestPower));
        motors[2].setPower((lbPower / highestPower));
        motors[3].setPower((rbPower / highestPower));
    }
}
