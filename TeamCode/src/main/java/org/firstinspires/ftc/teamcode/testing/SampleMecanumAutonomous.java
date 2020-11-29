package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class SampleMecanumAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Our drive base
        SampleMecanumDriveBase robot = new SampleMecanumDriveBase("rev", "hdhex", 20, 1, 17, 15);

        // Initialize hardware
        robot.initialize(true);

        // Waiting for the user to press play
        waitForStart();

        // Moves the robot forward 50 inches
        robot.forward(50);
    }
}

