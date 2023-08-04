package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class FunOpMode extends LinearOpMode {
    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

//        Trajectory forwardRightTrajectory = drive.trajectoryBuilder(new Pose2d())
//                .lineToLinearHeading(new Pose2d(60.0, 0.0, Math.toRadians(-90)))
//                .build();

        Pose2d[] targetPositions = {
                // Square
//                new Pose2d(60, 0, Math.toRadians(-90)),
//                new Pose2d(60, -60, Math.toRadians(180)),
//                new Pose2d(0, -60, Math.toRadians(90)),
//                new Pose2d(0, 0, Math.toRadians(0)),
                // Star
                new Pose2d(0, -60, Math.toRadians(180)),
                new Pose2d(-35.26, -11.46, Math.toRadians(360)),
                new Pose2d(18, -30, Math.toRadians(180)),
                new Pose2d(-36.26, -48.54, Math.toRadians(360)),
                new Pose2d(0,0, Math.toRadians(180)),
        };

        waitForStart();

        if (isStopRequested()) return;

        for(int i = 0; i < targetPositions.length; i++){
            Pose2d pose = targetPositions[i];
            Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(pose)
                    .build();
            drive.followTrajectory(trajectory);
        }

        // Final telemetry
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
