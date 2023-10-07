package org.firstinspires.ftc.teamcode.CameraCode;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

//Pipeline Class - does not include camera initialization.
public class PropDetectionPipeline implements VisionProcessor {
    private String location = "nothing"; // output

    //the color we are testing for - can be a range of colors(but only from min to max)
    /**
     * FOR SCALING SCALAR VALUES:
     * HSV stands for hue, saturation,
     * and value, hue (the 0-360° thing in google, but in openCV it's 0-180)
     * saturation (0-100% in google, but 0-255 in openCV)
     * and value (same as saturation in google/openCV)
     *
     */
    public Scalar lower = new Scalar(0, 0, 0); // HSV threshold bounds
    public Scalar upper = new Scalar(255, 255, 255);
    private Mat hsvMat = new Mat(); // converted image
    private Mat binaryMat = new Mat(); // image analyzed after thresholding
    private Mat maskedInputMat = new Mat();
    // Rectangle regions to be scanned
    private Point topLeft1 = new Point(10, 0), bottomRight1 = new Point(40, 20);
    private Point topLeft2 = new Point(10, 0), bottomRight2 = new Point(40, 20);
    private Point topLeft3 = new Point(10, 0), bottomRight3 = new Point(40, 20);

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }
    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        // Convert from BGR to HSV

        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat, lower, upper, binaryMat);
        // Scan both rectangle regions, keeping track of how many
        // pixels meet the threshold value, indicated by the color white
        // in the binary image
     // /*

        double w1 = 0, w2 = 0, w3 = 0;
        // process the pixel value for each rectangle  (255 = W, 0 = B)
      //  Mat left = frame.submat( new Rect( topLeft1, bottomRight1 ) );
      //  w1 = Core.sumElems( left ).val[0];

        for (int i = (int) topLeft2.x; i <= bottomRight2.x; i++) {
            for (int j = (int) topLeft2.y; j <= bottomRight2.y; j++) {
                if (binaryMat.get(i, j)[0] == 255) {
                    w2++;
                }
            }
        }

        for (int i = (int) topLeft3.x; i <= bottomRight3.x; i ++){
            for(int j = (int) topLeft3.y; i <= topLeft3.y; j ++){
                if(binaryMat.get(i, j)[0] == 255){
                    w3++;
                }
            }
        }
        // Determine object location
        //directionals are relative to the bot position - which means I will need more than one auto and camera.
        if (w1 > w2) {
            location = "leftMark";
        } else if (w1 < w2) {
            location = "middleMark";
        } else if((w3 > w1)){
            location = "rightMark";
        }
        binaryMat.copyTo(input);
        return null;
    }
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
//        Imgproc.rectangle(
//                maskedInputMat, // Buffer to draw on
//                topLeft1, // First point which defines the rectangle
//                bottomRight1, // Second point which defines the rectangle
//                new Scalar(0,0,255), // The color the rectangle is drawn in
//                1); // Thickness of the rectangle lines

    }
    public String getLocation() {
        return location;
    }

}