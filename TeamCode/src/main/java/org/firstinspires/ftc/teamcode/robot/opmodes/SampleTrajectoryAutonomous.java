package org.firstinspires.ftc.teamcode.robot.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.utility.PoseStorage;

@Autonomous(group = "Sample")
public class SampleTrajectoryAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Our drive base
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to start the bot at the bottom right red line
        Pose2d startPose = new Pose2d(-49.0, -49.0, Math.toRadians(0.0));
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(-48.00, -20.0))
                .splineToSplineHeading(new Pose2d(15.0, 3.0, Math.toRadians(75.0)), 4.0)
                .build();

        drive.followTrajectory(traj1);

        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
