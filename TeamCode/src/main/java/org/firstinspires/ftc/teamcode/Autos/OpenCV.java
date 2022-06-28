package org.firstinspires.ftc.teamcode.Autos;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
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
    /*
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

        */
    double z;
    double r;
    int ex;
    int ey;
    int ew;
    int eh;
    int cx;
    int cy;
    int cw;
    int ch;


    double lh;
    double ls;
    double lv;
    double hh;
    double hs;
    double hv;


    double minApproxLen;
    double maxApproxLen;



    Mat mat = new Mat();
    Mat outMat = new Mat();

    @Override
    public Mat processFrame(Mat input) {

        Rect rect = null;
        Rect rect_Crop = null;
        Mat gray = new Mat();
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);
        Mat edges = new Mat();
        Imgproc.Canny(gray, edges, r, z);
        Mat dilate = new Mat();
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size((2*2) + 1, (2*2)+1));
        Imgproc.dilate(edges, dilate, kernel);
        List<MatOfPoint> econtours = new ArrayList<>();
        Mat ehierarchy = new Mat();
        Imgproc.findContours(dilate, econtours, ehierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        for (MatOfPoint c: econtours){
            MatOfPoint2f c2f = new MatOfPoint2f(c.toArray());
            double peri = Imgproc.arcLength(c2f, true);
            MatOfPoint2f approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(c2f, approx, 0.02 * peri, true);
            Point[] points = approx.toArray();
            rect = Imgproc.boundingRect(c);
            ex = rect.x;
            ey = rect.y;
            ew = rect.width;
            eh = rect.height;
            Rect rectCrop = new Rect(ex, ey, ew, eh);
            Mat croppedFrame = new Mat(input, rectCrop);
            Scalar lowHSV = new Scalar(lh, ls, lv);
            Scalar highHSV = new Scalar(hh, hs, hv);

            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Mat mask = new Mat();
            Core.inRange(hsv, lowHSV, highHSV, mask);
            List<MatOfPoint> ccontours = new ArrayList<>();
            Mat chierarchy = new Mat();
            for (int j = 0; j < ccontours.size(); j++) {
                Mat contour = ccontours.get(j);
                double ccontourArea = Imgproc.contourArea(contour);
                rect = Imgproc.boundingRect(contour);
                cx = rect.x;
                cy = rect.y;
                cw = rect.width;
                ch = rect.height;



            }
            if(points.length > minApproxLen && points.length <= maxApproxLen){
                Point pt1 = new Point(cx, cy);
                Point pt2 = new Point(cw, ch);
                Scalar color = new Scalar(0, 0, 255);
                Imgproc.rectangle(outMat, pt1, pt2, color);
            }
        }
        return input;
    }
}
