package org.firstinspires.ftc.teamcode.robot.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group = "Sample")
public class SamplePathAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Our drive base
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to start the bot at the bottom right red line
        Pose2d startPose = new Pose2d(-63.0, -50.0, Math.toRadians(0.0));
        drive.setPoseEstimate(startPose);

        waitForStart();
        if (isStopRequested()) return;

        // Our trajectories
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(10.0, -48.0, Math.toRadians(-90.0)))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(-40.0, -10.0), 2.5)
                .build();

        // Run trajectories
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
    }
}
