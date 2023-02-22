package org.firstinspires.ftc.teamcode.auto.util;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

//for dashboard
/*@Config*/
public class PoleDetectionPipeline extends OpenCvPipeline {

    //these are public static to be tuned in dashboard
    public static double strictLowS = 140;
    public static double strictHighS = 255;
    private List<Rect> rects = new ArrayList<>();
    public Rect averageRect;

    public Mat processFrame(Mat input) {
        Mat mat = new Mat();

        //mat turns into HSV value
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }

        // lenient bounds will filter out near yellow, this should filter out all near yellow things(tune this if needed)
        Scalar lowHSV = new Scalar(20, 70, 80); // lenient lower bound HSV for yellow
        Scalar highHSV = new Scalar(32, 255, 255); // lenient higher bound HSV for yellow

        Mat thresh = new Mat();

        // Get a black and white image of yellow objects
        Core.inRange(mat, lowHSV, highHSV, thresh);

        Mat masked = new Mat();
        //color the white portion of thresh in with HSV from mat
        //output into masked
        Core.bitwise_and(mat, mat, masked, thresh);
        //calculate average HSV values of the white thresh values
        Scalar average = Core.mean(masked, thresh);

        Mat scaledMask = new Mat();
        //scale the average saturation to 150
        masked.convertTo(scaledMask, -1, 150 / average.val[1], 0);


        Mat scaledThresh = new Mat();
        //you probably want to tune this
        Scalar strictLowHSV = new Scalar(0, strictLowS, 0); //strict lower bound HSV for yellow
        Scalar strictHighHSV = new Scalar(255, strictHighS, 255); //strict higher bound HSV for yellow
        //apply strict HSV filter onto scaledMask to get rid of any yellow other than pole
        Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledThresh);

        Mat finalMask = new Mat();
        //color in scaledThresh with HSV, output into finalMask(only useful for showing result)(you can delete)
        Core.bitwise_and(mat, mat, finalMask, scaledThresh);

        Mat edges = new Mat();
        //detect edges(only useful for showing result)(you can delete)
        Imgproc.Canny(scaledThresh, edges, 100, 200);

        //contours, apply post processing to information
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        //find contours, input scaledThresh because it has hard edges
        Imgproc.findContours(scaledThresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // todo: analyze multiple frames to improve consistency

        // release color detection mats
        scaledThresh.release();
        scaledThresh.release();
        scaledMask.release();
        masked.release();
        edges.release();
        finalMask.release();
        hierarchy.release();
        mat.release();

        // Find largest connected component as a rectangle
        Mat labeled = new Mat(thresh.size(), thresh.type());
        Mat rectComponents = Mat.zeros(new Size(0, 0), 0);
        Mat centComponents = Mat.zeros(new Size(0, 0), 0);

        Imgproc.connectedComponentsWithStats(thresh, labeled, rectComponents, centComponents);

        // Collect regions info
        int[] rectangleInfo = new int[5];

        Rect largestRect = null;
        for(int i = 1; i < rectComponents.rows(); i++) {
            // Extract bounding box
            rectComponents.row(i).get(0, 0, rectangleInfo);
            Rect rectangle = new Rect(rectangleInfo[0], rectangleInfo[1], rectangleInfo[2], rectangleInfo[3]);
            if (largestRect == null || rectangle.area() > largestRect.area()) {
                largestRect = rectangle;
            }
        }

        if (largestRect != null) {
            rects.add(largestRect);
            if (rects.size() > 5) {
                rects.remove(0);
            }
            findAverageRect();
            Rect avg = averageRect;
            if (avg != null && avg.width != 0 && avg.height > 300) {
                int center = (int) (724 - 2.69 * avg.width);
                double offsetX = (int) (40 * (avg.x + avg.width/2 - center) / avg.width);
                double diff = avg.width - 160;
                diff -= offsetX < 0 ? offsetX / 20d : offsetX / 4d;
                double linearTerm = -diff * 2.4;
                double quadratic = 0.001 * Math.pow(diff, 2);
                // todo: not working when diff << 0
                double offsetY = (int) (linearTerm + quadratic);
                OpModeHolder.opMode.telemetry.addData("Rx", avg.x);
                OpModeHolder.opMode.telemetry.addData("Ry", avg.y);
                OpModeHolder.opMode.telemetry.addData("Rw", avg.width);
                OpModeHolder.opMode.telemetry.addData("Rh", avg.height);
                OpModeHolder.opMode.telemetry.addData("Rmid", avg.x + avg.width/2);
                OpModeHolder.opMode.telemetry.addData("center", center);
                OpModeHolder.opMode.telemetry.addData("linear", linearTerm);
                OpModeHolder.opMode.telemetry.addData("quadratic", quadratic);
                OpModeHolder.opMode.telemetry.addData("total", quadratic + linearTerm);
                OpModeHolder.opMode.telemetry.addData("offsetX", offsetX);
                OpModeHolder.opMode.telemetry.addData("offsetY", offsetY);
                OpModeHolder.opMode.telemetry.addData("diff", diff);
                OpModeHolder.opMode.telemetry.update();
            }
        } else if (!rects.isEmpty()) {
            rects.remove(0);
        }

        rectComponents.release();
        centComponents.release();
        labeled.release();
        thresh.release();


        //change the return to whatever mat you want
        //for example, if I want to look at the lenient thresh:
        // return thresh;
        // note that you must not do thresh.release() if you want to return thresh
        // you also need to release the input if you return thresh(release as much as possible)
//        return contours.size() != 0 ? contours.get(0) : finalMask;
        return input;
    }

    public void findAverageRect() {
        double x = 0;
        double y = 0;
        double w = 0;
        double h = 0;
        for (Rect r : rects) {
            x += r.x;
            y += r.y;
            w += r.width;
            h += r.height;
        }
        if (!rects.isEmpty()) {
            x /= rects.size();
            y /= rects.size();
            w /= rects.size();
            h /= rects.size();
        }
        averageRect = new Rect((int) x, (int) y, (int) w, (int) h);
    }

}
