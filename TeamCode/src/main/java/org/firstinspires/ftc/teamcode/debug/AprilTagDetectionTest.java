package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.autonomous.ParkingLocation;
import org.firstinspires.ftc.teamcode.opmode.autonomous.ParkingLocationAnalyzer;

@TeleOp(name="April Tag Detection Test", group="Autonomous")
public class AprilTagDetectionTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        ParkingLocationAnalyzer parkingLocationAnalyzer = new ParkingLocationAnalyzer(hardwareMap);
        ParkingLocation currentTargetParkingLocation, bestParkingLocation = null;
        boolean tagFound = false;

        // vvv The following loop replaces `waitForStart()`. vvv
        while (!isStarted() && !isStopRequested()) {
            currentTargetParkingLocation = parkingLocationAnalyzer.getParkingLocation();

            if (currentTargetParkingLocation != null) {
                tagFound            = true;
                bestParkingLocation = currentTargetParkingLocation;
            }

            if (tagFound && currentTargetParkingLocation != null) {
                telemetry.addData("Tag in sight: ", bestParkingLocation);
            } else if (tagFound) {
                telemetry.addData("Tag out of sight; going to: ", bestParkingLocation);
            } else {
                telemetry.addLine("No tag in sight; never seen a tag");
            }

            telemetry.update();
            sleep(20); // Only attempt to get an image every 20ms.
        }
        // ^^^ End of waitForStart() replacement. ^^^

        if (tagFound) {
            telemetry.addData("Tag snapshot: ", bestParkingLocation);
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
        }

        // Prevent the OpMode from ending.
        while (opModeIsActive()) {
            sleep(1000);
        }
    }
}
