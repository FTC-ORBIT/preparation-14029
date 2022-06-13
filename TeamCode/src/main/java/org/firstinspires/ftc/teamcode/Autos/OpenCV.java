package org.firstinspires.ftc.teamcode.Autos;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class OpenCV extends OpenCvPipeline {
    OpenCV(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    Telemetry telemetry;
    public static double lowBlueShH;
    public static double highBlueShH;
    public static double lowBlueShS;
    public static double highBlueShS;
    public static double lowBlueShV;
    public static double highBlueShV;
    static double bx,by;
    static double rx, ry;
    static double sx, sy;
    static double cx, cy;
    static double bax, bay;
    double contour1 = 0;
    double contour2 = 0;


    Mat mat = new Mat();


    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Scalar lowBlueShHSV = new Scalar(lowBlueShH, lowBlueShS, lowBlueShV);
        Scalar highBlueShHSV = new Scalar(highBlueShH, highBlueShS, highBlueShV);
        Mat thresh = new Mat();
        Core.inRange(mat, lowBlueShHSV, highBlueShHSV, thresh);

        Mat edges = new Mat;
        Imgproc.Canny(thresh, edges, 100, 300);
        //List<MatOfPoint> contours = new ArrayList<>();
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        hierarchy.release();
        double contourX = -1;
        double maxArea = -1;
        Rect maxAreaRect = null;

        for(MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            maxArea = Math.max(area, maxArea);
            telemetry.addData("area",area);
            if(area==maxArea && area > 0){
                maxAreaRect = Imgproc.boundingRect(contour);
                Moments M = Imgproc.moments(contour);
                contour1 = M.m10 / M.m00;
                contour2 = M.m01 / M.m00
                bx, by = contour1, contour2;

            }


        }
        if (maxAreaRect != null) {
            Imgproc.rectangle(input, new Point(maxAreaRect.x, maxAreaRect.y), new Point(maxAreaRect.x + maxAreaRect.width, maxAreaRect.y + maxAreaRect.height), new Scalar(255, 0, 0), 2);

        }


        return input;


    }



    public static double blueShPos(){
        return bx ;


    }
    public static double redShPos(){
        return rx;
        return ry;

    }
    public static double sharedShPos(){
        return sx, return sy;

    }
    public static double cubePos(){
        return cx, return cy;

    }
    public static double blueShPos(){
        return bax, return bay;

    }


}
