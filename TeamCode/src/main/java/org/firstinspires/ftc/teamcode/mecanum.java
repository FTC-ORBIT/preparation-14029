package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class mecanum extends OpMode {
    public final DcMotor[] motors = new DcMotor[4];
    public BNO055IMU imu;
    private float offSet = 0;
    private OpMode opMode;

    double[] motorsLast = new double[4];
    double motory =0;

    ElapsedTime timer = new ElapsedTime();

    private double distanceFactor = 8.5;
    private double distanceFactorXydrive = 8.5;

    private double speedFactor = 1.0;
    public void setSpeedFactor(double speedFactor) {
        this.speedFactor = speedFactor;
    }

    public void init() {
        opMode.telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        motors[0] = opMode.hardwareMap.get(DcMotor.class, "lf");
        motors[1] = opMode.hardwareMap.get(DcMotor.class, "rf");
        motors[2] = opMode.hardwareMap.get(DcMotor.class, "lb");
        motors[3] = opMode.hardwareMap.get(DcMotor.class, "rb");
        for (final DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {

    }



    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void fieldCentric(final float y,final float x,final double r) {
        final Vector joystickVector = new Vector(x, y);
        final float robotAngle = (float) Math.toRadians(getAngle());
        final Vector rotated = joystickVector.rotate(robotAngle);
        drive(rotated.y, rotated.x, r);
    }

    public void motortest() {
        while (true) {
            opMode.telemetry.addData("lf", motors[0].getCurrentPosition());
            opMode.telemetry.addData("rf", motors[1].getCurrentPosition());
            opMode.telemetry.addData("lb", motors[2].getCurrentPosition());
            opMode.telemetry.addData("tb", motors[3].getCurrentPosition());
            opMode.telemetry.update();
        }
    }
    public void drive(double y, double x, double r) {
        final double lfPower = y + x + r;
        final double rfPower = y - x + r;
        final double lbPower = y - x - r;
        final double rbPower = y + x - r;
        double highestPower = 1;
        final double max = Math.max(Math.abs(lfPower), Math.max(Math.abs(lbPower), Math.max(Math.abs(rfPower), Math.abs(rbPower))));
        if (max > 1) highestPower = max;
        motors[0].setPower((lfPower / highestPower) * speedFactor);
        motors[1].setPower((lbPower / highestPower) * speedFactor);
        motors[2].setPower((rfPower / highestPower) * speedFactor);
        motors[3].setPower((rbPower / highestPower) * speedFactor);
    }
}
