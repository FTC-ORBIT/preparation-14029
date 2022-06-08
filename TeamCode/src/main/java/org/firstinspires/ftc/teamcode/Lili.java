package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "PID");
public class Lili extends LinearOpMode {
    double deltaTIme=0;
    double deltaAngle =0;
    double lastTIme =0;
    double lastAngle =0;
    double I=0;
    private DcMotor rf;
    private DcMotor rb;
    private DcMotor lf;
    private DcMotor lb;
    ElapsedTime time = new ElapsedTime();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private BNO055IMU imu;


    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry = dashboard.getTelemetry();
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

        waitForStart();

        driveRobot(90);
    }
    public double PID(double wantedAngle){
        //P
        double error = wantedAngle - getAngle();
        deltaTIme = time.milliseconds() - lastTIme;
        deltaAngle = error - lastAngle;
        double D = deltaTIme == 0 ? 0 : deltaAngle/deltaTIme;
        I =+ error;
        if (Math.signum(lastAngle) != Math.signum(getAngle()))
            I=0;
        lastTIme = time.milliseconds();
        lastAngle = error;
        return (error * 0.4 + (I * 0) + (D * 0));
    }
    public double getAngle(){
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        return angle;
    }
    public void driveRobot(double wantedAngle) {
        double lMotorsPower = (-gamepad1.left_stick_y + gamepad1.right_trigger - PID(wantedAngle));
        double rMotorsPower = (-gamepad1.left_stick_y + gamepad1.left_trigger + PID(wantedAngle));

//        final float highestPower = Math.max( Math.abs(lMotorsPower), Math.abs(rMotorsPower) );
//        lMotorsPower/=highestPower;
//        rMotorsPower/=highestPower;
        lf.setPower(lMotorsPower);
        lb.setPower(lMotorsPower);
        rf.setPower(rMotorsPower);
        rb.setPower(rMotorsPower);
    }

}
