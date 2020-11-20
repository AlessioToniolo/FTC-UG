package org.firstinspires.ftc.teamcode.robot.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.utility.BaseRobot;
import org.firstinspires.ftc.teamcode.robot.utility.PoseStorage;

/**
 * If A is pressed, the bot will generate a splineTo() trajectory on the fly and follow it to
 * targetA (x: 45, y: 45, heading: 90deg).
 * <p>
 * If B is pressed, the bot will generate a lineTo() trajectory on the fly and follow it to
 * targetB (x: -15, y: 25, heading: whatever the heading is when you press B).
 * <p>
 * If Y is pressed, the bot will turn to face 45 degrees, no matter its position on the field.
 * <p>
 * Pressing X will cancel trajectory following and switch control to the driver. The bot will also
 * cede control to the driver once trajectory following is done.
 * <p>
 * The following may be a little off with this method as the trajectory follower and turn
 * function assume the bot starts at rest.
 * <p>
 * This sample utilizes the SampleMecanumDrive.java class.
 */
@TeleOp(group = "Competition")
public class CompetitionPositionToggleCentric extends LinearOpMode {
    // Define 2 states, drive control or automatic control
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    // The location we want the bot to automatically go to when we press the B button
    Pose2d targetPose = new Pose2d(-10.0, -30.0, Math.toRadians(0.0));

    // Base Robot Fields
    BaseRobot robot   = new BaseRobot();
    double clawOffset  = 0.0;
    final double CLAW_SPEED  = 0.02;
    private ElapsedTime runtime = new ElapsedTime();

    // Toggle Fields
    boolean prevValueShooter = false;
    boolean toggleShooter = false;
    boolean prevValueBothIntake = false;
    boolean toggleBothIntake = false;
    boolean prevValueBothOutake = false;
    boolean toggleBothOutake = false;
    boolean prevValueSurgicalIntake = false;
    boolean toggleSurgicalIntake = false;
    boolean prevValueWheelIntake = false;
    boolean toggleWheelIntake = false;
    // Not currently in use
    boolean prevValueSpeedControl = false;
    boolean toggleSpeedControl = false;
    double speedCap = 0.5;
    // Speed Cap State Machine
    boolean speedCapOn = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize custom cancelable SampleMecanumDrive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Initialize custom mechanism manager class
        robot.initialize(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Message:", "Robot Ready");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Update the drive class
            drive.update();


            Pose2d poseEstimate = drive.getPoseEstimate();

            // TODO: NOT IN USE; Adjusted Pose Heading for teleop sides (program currently for left side of field if not used)
            double poseRight = -poseEstimate.getHeading() + 180;

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            // **Change line 104's heading to poseRight for field orientations**
            // TODO: add orientation logic
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            // We follow different logic based on whether we are in manual driver control or switch
            // control to the automatic mode
            switch (currentMode) {
                case DRIVER_CONTROL:

                    // TODO: debugging for speed control
                    telemetry.addData("power", input.getX());
                    telemetry.addData("power", input.getY());
                    telemetry.addData("power", -gamepad1.right_stick_x);
                    telemetry.update();

                    // Driving functionality
                    drive.setWeightedDrivePower(
                            // TODO: add speed control
                            new Pose2d(
                                    input.getX(),
                                    input.getY(),
                                    -gamepad1.right_stick_x
                            )
                    );

                    // Go To Position trajectory
                    if (gamepad1.b) {
                        // If the B button is pressed on gamepad1, we generate a lineToLinearHeading()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .lineToSplineHeading(targetPose)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }

                    // TODO: NOT IN USE; Mechanism Control

                    /*
                    // Wobble Arm
                    double armMotorPower = gamepad1.right_trigger - gamepad1.left_trigger;
                    if (armMotorPower > 0.4) {
                        armMotorPower = 0.4;
                    } else if (armMotorPower < -0.4) {
                        armMotorPower = -0.4;
                    }
                    robot.wobbleArm.setPower(armMotorPower);

                    // Wobble Servo
                    if (gamepad1.right_bumper) {
                        robot.leftHand.setPosition(1.0);
                    } else if (gamepad1.left_bumper) {
                        robot.leftHand.setPosition(0.2);
                    }

                    // Shooter with toggle
                    if (gamepad1.dpad_up && gamepad1.dpad_up != prevValueShooter) {
                        if (!toggleShooter){
                            // If inaccurate -0.77 is golden power, second power is -0.7785
                            robot.testMotor.setPower(-1);
                        } else {
                            robot.testMotor.setPower(0);
                        }
                        toggleShooter = !toggleShooter;
                    }
                    prevValueShooter = gamepad1.dpad_up;

                    // Both intake with toggle
                    if (gamepad1.dpad_left && gamepad1.dpad_left != prevValueBothIntake) {
                        if (!toggleBothIntake){
                            robot.surgicalTubingIntake.setPower(1);
                            robot.wheelIntake.setPower(1);
                        } else {
                            robot.surgicalTubingIntake.setPower(0);
                            robot.wheelIntake.setPower(0);
                        }
                        toggleBothIntake = !toggleBothIntake;
                    }
                    prevValueBothIntake = gamepad1.dpad_left;

                    // Both outake with toggle
                    if (gamepad1.dpad_right && gamepad1.dpad_right != prevValueBothOutake) {
                        if (!toggleBothOutake){
                            robot.surgicalTubingIntake.setPower(-1);
                            robot.wheelIntake.setPower(-1);
                        } else {
                            robot.surgicalTubingIntake.setPower(0);
                            robot.wheelIntake.setPower(0);
                        }
                        toggleBothOutake = !toggleBothOutake;
                    }
                    prevValueBothOutake = gamepad1.dpad_right;

                    // Surgical tubing intake with toggle
                    if (gamepad1.dpad_down && gamepad1.dpad_down != prevValueSurgicalIntake) {
                        if (!toggleSurgicalIntake){
                            robot.surgicalTubingIntake.setPower(1);
                        } else {
                            robot.surgicalTubingIntake.setPower(0);
                        }
                        toggleSurgicalIntake = !toggleSurgicalIntake;
                    }
                    prevValueSurgicalIntake = gamepad1.dpad_down;
                    */


                case AUTOMATIC_CONTROL:
                    // If a is pressed, we break out of the automatic following
                    if (gamepad1.a) {
                        drive.cancelFollowing();
                        currentMode = Mode.DRIVER_CONTROL;
                    }

                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }
        }
    }
}