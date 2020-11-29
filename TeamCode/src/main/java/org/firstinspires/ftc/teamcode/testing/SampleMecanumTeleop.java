package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class SampleMecanumTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Drive base
        SampleMecanumDriveBase robot = new SampleMecanumDriveBase("rev", "hdhex", 20, 1, 17, 15);

        // Initialize hardware
        robot.initialize(false);

        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {

            // Set controller powers for motors
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.07; // To correct imperfect strafing
            double rx = gamepad1.right_stick_x;

            double frontLeftPower = y + x + rx;
            double frontRightPower = y - x - rx;
            double backLeftPower = y - x + rx;
            double backRightPower = y + x - rx;

            // Put powers in the range of -1 to 1 only if they aren't already (not
            // checking would cause us to always drive at full speed)
            if (Math.abs(frontLeftPower) > 1 || Math.abs(backLeftPower) > 1 ||
                    Math.abs(frontRightPower) > 1 || Math.abs(backRightPower) > 1 ) {
                // Find the largest power
                double max = 0;
                max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
                max = Math.max(Math.abs(frontRightPower), max);
                max = Math.max(Math.abs(backRightPower), max);

                // Divide everything by max (it's positive so we don't need to worry
                // about signs)
                frontLeftPower /= max;
                backLeftPower /= max;
                frontRightPower /= max;
                backRightPower /= max;
            }

            // Set motor powers
            robot.lf.setPower(frontLeftPower);
            robot.lr.setPower(backLeftPower);
            robot.rf.setPower(frontRightPower);
            robot.rr.setPower(backRightPower);
        }
    }
}
