package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

// Import axesreference and angles
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;

@Autonomous(name="Autonomous Parking", group="Autonomous")
public class AutoParking extends LinearOpMode {
    private static final double TILE_METER_LENGTH = 0.6;

    // Used for resetting the orientation of the robot throughout the parking process.
    BNO055IMU imu;

    // NOTE: These values are calibration for determining the distance to the image
    // and are not needed for the Power Play season of FTC.
    // All units are in meters.
    static final double FEET_PER_METER = 3.28084;
    private double fx = 578.272;
    private double fy = 578.272;
    private double cx = 402.145;
    private double cy = 221.506;
    private double tagsize = 0.166;

    // Tags to look for 1, 2, 3 for zones 1, 2, 3.
    private final int LEFT_TAG_ID   = 1;
    private final int CENTER_TAG_ID = 2;
    private final int RIGHT_TAG_ID  = 3;

    private AprilTagDetection tagOfInterest = null;

    private void resetHeading(DcMotor[] driveMotors) {
        final int sleepBetweenStepTime = 250;
        final double headingCorrectionPower = 0.05;
        double botHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        sleep(sleepBetweenStepTime);

        while (opModeIsActive() && botHeading != 0) {
            botHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            if (botHeading > 0) {
                driveMotors[0].setPower(headingCorrectionPower);
                driveMotors[1].setPower(headingCorrectionPower);
                driveMotors[2].setPower(-headingCorrectionPower);
                driveMotors[3].setPower(-headingCorrectionPower);
            } else if (botHeading < 0) {
                driveMotors[0].setPower(-headingCorrectionPower);
                driveMotors[1].setPower(-headingCorrectionPower);
                driveMotors[2].setPower(headingCorrectionPower);
                driveMotors[3].setPower(headingCorrectionPower);
            }
        }

        for (DcMotor motor : driveMotors) {
            motor.setPower(0);
        }

        sleep(sleepBetweenStepTime);
    }

    private void driveForwardOneTile(DcMotor[] driveMotors) {
        // NOTE: Currently drives without the encoder.
        // Configured for Rev Robotics HD Hex Motor 20:1
        // 537.6 ticks per revolution
        // 96mm diameter wheel
        final double drivePower = 0.2;

        for (DcMotor motor : driveMotors) {
            motor.setPower(drivePower);
        }

        sleep((long)(TILE_METER_LENGTH / drivePower * 1100));

        for (DcMotor motor : driveMotors) {
            motor.setPower(0);
        }

        // Drive backwards slightly to clear the cone that we just pushed out of the way.
        for (DcMotor motor : driveMotors) {
            motor.setPower(-drivePower);
        }

        sleep((long)(TILE_METER_LENGTH / drivePower * 250));

        for (DcMotor motor : driveMotors) {
            motor.setPower(0);
        }
    }

    private void strafeLeftOneTile(DcMotor[] mecanumMotors) {
        // NOTE: Currently drives without the encoder.
        // Configured for Rev Robotics HD Hex Motor 20:1
        // 537.6 ticks per revolution
        // 96mm diameter wheel
        final double drivePower = 0.15;

        mecanumMotors[0].setPower(-drivePower);
        mecanumMotors[1].setPower(drivePower);
        mecanumMotors[2].setPower(drivePower);
        mecanumMotors[3].setPower(-drivePower);

        sleep((long)(TILE_METER_LENGTH / drivePower * 1000));

        for (DcMotor motor : mecanumMotors) {
            motor.setPower(0);
        }
    }

    private void strafeRightOneTile(DcMotor[] mecanumMotors) {
        // NOTE: Currently drives without the encoder.
        // Configured for Rev Robotics HD Hex Motor 20:1
        // 537.6 ticks per revolution
        // 96mm diameter wheel
        final double drivePower = 0.15;

        mecanumMotors[0].setPower(drivePower);
        mecanumMotors[1].setPower(-drivePower);
        mecanumMotors[2].setPower(-drivePower);
        mecanumMotors[3].setPower(drivePower);

        sleep((long)(TILE_METER_LENGTH / drivePower * 1000));

        for (DcMotor motor : mecanumMotors) {
            motor.setPower(0);
        }
    }

    @Override
    public void runOpMode() {
        DcMotor[] mecanumMotors = {
            hardwareMap.get(DcMotor.class, "motorFrontLeft"),
            hardwareMap.get(DcMotor.class, "motorBackLeft"),
            hardwareMap.get(DcMotor.class, "motorFrontRight"),
            hardwareMap.get(DcMotor.class, "motorBackRight")
        };

        // Reverse left side motors.
        mecanumMotors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        mecanumMotors[1].setDirection(DcMotorSimple.Direction.REVERSE);

        // Init IMU. Used for resetting the heading at the end of the parking sequence.
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // Init camera setup.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
        AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        // Attach the pipeline to the camera.
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", errorCode);
                telemetry.update();
            }
        });

        telemetry.setMsTransmissionInterval(50);

        // vvv The following loop replaces `waitForStart()`. vvv
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT_TAG_ID || tag.id == CENTER_TAG_ID || tag.id == RIGHT_TAG_ID) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\nData:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest.");

                    if(tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
            } else {
                telemetry.addLine("Don't see tag of interest.");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen:");
                    tagToTelemetry(tagOfInterest);
                }
            }

            telemetry.update();
            sleep(20);
        }
        // ^^^ End of `waitForStart()` replacement. ^^^

        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop.");
            telemetry.update();

            // Drive to the middle as the default.
            // 1 / 3 chance of being correct.
            driveForwardOneTile(mecanumMotors);
        }

        if (tagOfInterest.id == LEFT_TAG_ID) {
            // Drive to the Left Zone.
            // This is the default behavior in case the tag is never seen and something goes wrong.
            driveForwardOneTile(mecanumMotors);
            resetHeading(mecanumMotors);
            strafeLeftOneTile(mecanumMotors);
        } else if (tagOfInterest.id == CENTER_TAG_ID) {
            // Drive to the Center Zone.
            driveForwardOneTile(mecanumMotors);
        } else if (tagOfInterest.id == RIGHT_TAG_ID) {
            // Drive to the Right Zone.
            driveForwardOneTile(mecanumMotors);
            resetHeading(mecanumMotors);
            strafeRightOneTile(mecanumMotors);
        }

        resetHeading(mecanumMotors);

        // Reset the heading, get the bot heading as close to 0 as possible using PID controls and the IMU.
        // double headingCorrectionPower = 0.07;
        // double botHeading = imu.getAngularOrientation().firstAngle;

        // while (opModeIsActive() && botHeading != 0) {
        //     botHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        //     if (botHeading > 0) {
        //         mecanumMotors[0].setPower(headingCorrectionPower);
        //         mecanumMotors[1].setPower(headingCorrectionPower);
        //         mecanumMotors[2].setPower(-headingCorrectionPower);
        //         mecanumMotors[3].setPower(-headingCorrectionPower);
        //     } else if (botHeading < 0) {
        //         mecanumMotors[0].setPower(-headingCorrectionPower);
        //         mecanumMotors[1].setPower(-headingCorrectionPower);
        //         mecanumMotors[2].setPower(headingCorrectionPower);
        //         mecanumMotors[3].setPower(headingCorrectionPower);
        //     }
        // }

        // while (opModeIsActive() && botHeading != 0) {
        //     if (botHeading > 0) { // Turn left.
        //         mecanumMotors[0].setPower(headingCorrectionPower);
        //         mecanumMotors[1].setPower(headingCorrectionPower);
        //         mecanumMotors[2].setPower(-headingCorrectionPower);
        //         mecanumMotors[3].setPower(-headingCorrectionPower);
        //     } else { // Turn right.
        //         mecanumMotors[0].setPower(-headingCorrectionPower);
        //         mecanumMotors[1].setPower(-headingCorrectionPower);
        //         mecanumMotors[2].setPower(headingCorrectionPower);
        //         mecanumMotors[3].setPower(headingCorrectionPower);
        //     }

        //     botHeading = imu.getAngularOrientation().firstAngle;
        // }

        for (DcMotor motor : mecanumMotors) {
            motor.setPower(0);
        }
    }

    private void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}
