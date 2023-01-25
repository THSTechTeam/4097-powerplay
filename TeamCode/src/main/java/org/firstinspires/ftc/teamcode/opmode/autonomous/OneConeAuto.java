//package org.firstinspires.ftc.teamcode.opmode.autonomous;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
//
//import static org.firstinspires.ftc.teamcode.DriveConstants.ONE_TILE_DISTANCE;
//
//@Autonomous(name="One Cone + Park", group="Autonomous")
//public class OneConeAuto extends LinearOpMode {
//    private ParkingLocationAnalyzer parkingLocationAnalyzer;
//    private ParkingLocation parkingLocation;
//
//    public static double forwardOffset = 0;
//    public static Pose2d scoreConePose = new Pose2d(0, 0, 0);
//    public static Pose2d resetPose = new Pose2d(0, 0, 0);
//
//    private TrajectorySequence scoreConeOne;
//    private TrajectorySequence scoreConeTwo;
//    private TrajectorySequence reset;
//
//    private TrajectorySequence leftParking;
//    private TrajectorySequence centerParking;
//    private TrajectorySequence rightParking;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        parkingLocationAnalyzer = new ParkingLocationAnalyzer(hardwareMap);
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        drive.setPoseEstimate(new Pose2d());
//
//        scoreConeOne = drive.trajectorySequenceBuilder(new Pose2d())
//                .forward(forwardOffset)
//                .addDisplacementMarker(() -> {
//                    drive.followTrajectorySequence(scoreConeTwo);
//                })
//                .build();
//
//        scoreConeTwo = drive.trajectorySequenceBuilder(scoreConeOne.end())
//                .splineTo(scoreConePose, 0)
//
//    }
//}
