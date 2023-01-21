package org.firstinspires.ftc.teamcode.auto.util;

import com.chsrobotics.ftccore.hardware.HardwareManager;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class AprilTagSleeveDetector {

    public static boolean started = false;
    public static Zone zone;
    public static OpenCvCamera camera;
    private static AprilTagDetectionPipeline pipeline;

    private static double fx = 578.272;
    private static double fy = 578.272;
    private static double cx = 402.145;
    private static double cy = 221.506;

    // UNITS ARE METERS
    static double tagsize = 0.041;
    private static int numFramesWithoutDetection = 0;

    static final float DECIMATION_HIGH = 3;
    static final float DECIMATION_LOW = 2;
    static final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    static final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    public static void initAprilTags(HardwareManager manager)
    {
        int cameraMonitorViewId = manager.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", manager.hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(manager.accessoryCameras[0], cameraMonitorViewId);
        pipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    public static void detect(HardwareManager manager) {
        ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();


        if(currentDetections != null)
        {
            manager.opMode.telemetry.addData("FPS", camera.getFps());
            manager.opMode.telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
            manager.opMode.telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

            // If we don't see any tags
            if(currentDetections.size() == 0)
            {
                numFramesWithoutDetection++;

                // If we haven't seen a tag for a few frames, lower the decimation
                // so we can hopefully pick one up if we're e.g. far back
                if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                {
                    pipeline.setDecimation(DECIMATION_LOW);
                }
            }
            // We do see tags!
            else
            {
                numFramesWithoutDetection = 0;

                // If the target is within 1 meter, turn on high decimation to
                // increase the frame rate
                if(currentDetections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                {
                    pipeline.setDecimation(DECIMATION_HIGH);
                }

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == 0) {
                        zone = Zone.ZONE_ONE;
                    } else if (tag.id == 1) {
                        zone = Zone.ZONE_TWO;
                    } else if (tag.id == 2) {
                        zone = Zone.ZONE_THREE;
                    }
                }
            }
        }
    }

    public enum Zone
    {
        ZONE_ONE,
        ZONE_TWO,
        ZONE_THREE
    }

}
