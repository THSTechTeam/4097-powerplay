package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class ParkingLocationAnalyzer {
    private final OpenCvCamera camera;
    private final AprilTagDetectionPipeline aprilTagDetectionPipeline;

    private static final int CAMERA_WIDTH  = 640;
    private static final int CAMERA_HEIGHT = 480;

    // NOTE: These values are calibration for determining the distance to the image.
    // These values are not needed for the Power Play season of FTC and as such are simply fillers.
    private static final double fx      = 578.272;
    private static final double fy      = 578.272;
    private static final double cx      = 402.145;
    private static final double cy      = 221.506;
    private static final double tagSize = 0.166;

    public enum ParkingLocation {
        LEFT,
        CENTER,
        RIGHT
    }

    public ParkingLocationAnalyzer(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap
                                    .appContext
                                    .getResources()
                                    .getIdentifier(
                                        "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory
            .getInstance()
            .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // Swallow the error. Required by the interface but never expected to be called.
            }
        });
    }

    public ParkingLocation getParkingLocation() {
        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getLatestDetections();

        if (detections.size() == 0) {
            return null;
        }

        ParkingLocation currentParkingLocation = null;

        for (AprilTagDetection detection : detections) {
            switch (detection.id) {
                case 1:
                    currentParkingLocation = ParkingLocation.LEFT;
                    break;
                case 2:
                    currentParkingLocation = ParkingLocation.CENTER;
                    break;
                case 3:
                    currentParkingLocation = ParkingLocation.RIGHT;
                    break;
            }
        }

        return currentParkingLocation;
    }
}
