package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
public class SampleMecanumAutonomousDash extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Our drive base
        SampleMecanumDriveBaseDash robot = new SampleMecanumDriveBaseDash("rev", "hdhex", 20, 1, 17, 15);

        // Initialize hardware
        robot.initialize(true);

        // Waiting for the user to press play
        waitForStart();

        // Moves the robot forward 50 inches
        robot.forward(50);
    }
}
