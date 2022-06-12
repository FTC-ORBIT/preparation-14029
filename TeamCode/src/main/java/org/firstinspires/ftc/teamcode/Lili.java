package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous (name="test")
public class Lili extends LinearOpMode {
    double deltaTIme=0;
    double[]deltaAngle =new double [5];
    double lastTIme =0;
    double lastAngle =0;
    double I=0;
    private DcMotor rf;
    private DcMotor rb;
    private DcMotor lf;
    private DcMotor lb;
    double predict =0;
    double[] deltaTimesAll = new double[5];
    double[] deltaInDers = new double[5];
    double[] firstOrderDerivatices = new double[5];
    double secondOrderDerivaties =0;
    double[] firstTimes = new double[4];
    ElapsedTime time = new ElapsedTime();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Constans constans = new Constans();
    int i=0;
    public BNO055IMU imu;
    public final DcMotor[] motors = new DcMotor[4];

    public void runOpMode() throws InterruptedException {
        final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
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
        motors[0] = hardwareMap.get(DcMotor.class, "lf");
        motors[1] = hardwareMap.get(DcMotor.class, "rf");
        motors[2] = hardwareMap.get(DcMotor.class, "lb");
        motors[3] = hardwareMap.get(DcMotor.class, "rb");

        waitForStart();
        TurnGyro(90);
    }
    void TurnGyro(double wantedAngle){
        while (Math.abs(wantedAngle - getAngle()) >= 1) {
                driveRobot(0,0,PID(wantedAngle));
                telemetry.addData("gyro", getAngle());
                telemetry.update();
        }
    }
    public double PID(double wantedAngle){
        //p
        double error = wantedAngle - getAngle();
        deltaTIme = time.milliseconds() - lastTIme;
        deltaAngle[i] = error - lastAngle;
        double D = deltaTIme == 0 ? 0 : deltaAngle[i]/deltaTIme;
        // to restart the taylor series inputs in the array
        if (i==4)
            i=0;
        //putting useful info into the array for later use
        deltaTimesAll[i] = deltaTIme;
        I =+ error;
        if (Math.signum(lastAngle) != Math.signum(getAngle()))
            I=0;
        lastTIme = time.milliseconds();
        lastAngle = error;
        double feedForward = feedForward(deltaTimesAll,deltaAngle,D);
        telemetry.addData("D",D);
        telemetry.addData("error",error);
        telemetry.addData("deltatime",deltaTIme);
        telemetry.addData("feed",feedForward);
        telemetry.addData("I",i);
        telemetry.addData("second order der",secondOrderDerivaties);
        telemetry.addData("der",D);
        //i++ for arrays in the taylor series
        i++;
        return (error * constans.Kp) +
                (I * constans.Ki) +
                + (feedForward * constans.feedForwardK);
    }
    public double getAngle(){
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        return angle;
    }
    public void driveRobot(double y,double x,double r) {
        /*double lMotorsPower = (- PID(wantedAngle));
        double rMotorsPower = (PID(wantedAngle));

//        final float highestPower = Math.max( Math.abs(lMotorsPower), Math.abs(rMotorsPower) );
//        lMotorsPower/=highestPower;
//        rMotorsPower/=highestPower;
        lf.setPower(lMotorsPower);
        lb.setPower(lMotorsPower);
        rf.setPower(rMotorsPower);
        rb.setPower(rMotorsPower);
         */
        final double lfPower = y + x + r;
        final double rfPower = y - x + r;
        final double lbPower = y - x - r;
        final double rbPower = y + x - r;
        double highestPower = 1;
        final double max = Math.max(Math.abs(lfPower), Math.max(Math.abs(lbPower), Math.max(Math.abs(rfPower), Math.abs(rbPower))));
        if (max > 1) highestPower = max;
        motors[0].setPower((lfPower / highestPower) * constans.speedFactor);
        motors[1].setPower((lbPower / highestPower) * constans.speedFactor);
        motors[2].setPower((rfPower / highestPower) * constans.speedFactor);
        motors[3].setPower((rbPower / highestPower) * constans.speedFactor);
    }
    public double feedForward(double[] delta,double[] angles,double oneDer){
        // the taylor series uses adding derivaties togather. the algo works with the first one.
        if (i==0) {
            firstTimes[0] = time.milliseconds();
            firstTimes[1] = angles[i] / (delta[i] + 0.005);
        }
        // each 2 runs it will add the derivatives. the allows us to do higher order derivaties as shown in taylor series
        if (i%2 !=0){
            firstOrderDerivatices[i%2] = oneDer;
            deltaInDers[i%2] = time.milliseconds();
        }
        // same process but this time not storage, but caculation of second order derivative
        if (i%4!=0) {
            secondOrderDerivaties = firstOrderDerivatices[2] - firstOrderDerivatices[(1)] / (deltaInDers[2] - deltaInDers[1]);
        }
        // adding it all toagather for taylor use
        predict = firstTimes[1] * (time.milliseconds() - firstTimes[0]+0.005) + (secondOrderDerivaties/2) * Math.pow(time.milliseconds() - firstTimes[0]+0.005,2);
        return predict;
    }
}
