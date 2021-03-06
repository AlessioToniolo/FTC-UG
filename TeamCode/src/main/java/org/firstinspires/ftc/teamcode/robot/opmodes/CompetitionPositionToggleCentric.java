package org.firstinspires.ftc.teamcode.robot.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.utility.BaseRobot;
import org.firstinspires.ftc.teamcode.robot.utility.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.utility.RobotAutomation;

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

    // Dashboard for testing
    FtcDashboard dashboard;

    Mode currentMode = Mode.DRIVER_CONTROL;

    // The location we want the bot to automatically go to when we press the B button
    Pose2d targetPose = new Pose2d(-10.0, -30.0, Math.toRadians(0.0));
    // The orientation we want the bot to turn to when we press the X button
    double targetAngle = Math.toRadians(0);

    // Base Robot Fields
    // TODO: bugs
    //BaseRobot robot   = new BaseRobot();
    double clawOffset  = 0.0;
    final double CLAW_SPEED  = 0.02;
    private ElapsedTime runtime = new ElapsedTime();

    // Rotated Vector Field
    double poseRight = 0;

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

    // Kicker Servo Fields
    int servoCount = 0;
    boolean isKicking = false;

    // Mechanism positions for dashboard
    public static double wobbleRightBumperPos = 1.0;
    public static double wobbleLeftBumperPos = 0.2;
    public static double isKickingHopperPos = 0.5;
    public static double isNotKickingHopperPos = 1.0;
    public static double shooterPower = -0.8;
    public static double clawOverHopperPos = 250;
    public static double clawOverGroundPos = 50;
    public static double clawServoReleasePos = 1.0;
    public static double clawServoGrabPos = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize custom cancelable SampleMecanumDrive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Initialize custom mechanism base class
        BaseRobot robot = new BaseRobot();

        // Initialize custom automation class
        //RobotAutomation auto = new RobotAutomation();

        // Initialize hardware for custom mechanism base class
        robot.initialize(hardwareMap);

        // Initialize hardware for automation class
        //auto.initializeAutomation();

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        // Dashboard initialization
        dashboard = FtcDashboard.getInstance();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Message:", "Robot Ready");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Update the drive class
            drive.update();


            Pose2d poseEstimate = drive.getPoseEstimate();

            // Adjusted Pose Heading for teleop sides (program currently for left side of field if not used)
            // TODO: figure out pose heading needed with auto and in general
            poseRight = -poseEstimate.getHeading() + 180;

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            // **Change line 104's heading to poseRight for field orientations**
            // TODO: add orientation logic
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(poseRight);

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

                    // Go To Position trajectory automation
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

                    // Turn To Position automation
                    if (gamepad1.x) {
                        // If X is pressed, we turn the bot to the specified angle to reach
                        // targetAngle (0 degrees, orientation for the goal)

                        drive.turnAsync(Angle.normDelta(targetAngle - poseEstimate.getHeading()));

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }

                    // TODO: NOT IN USE; Mechanism Control

                    // Wobble Arm
                    double armMotorPower = gamepad1.right_trigger - gamepad1.left_trigger;
                    if (armMotorPower > 0.4) {
                        armMotorPower = 0.4;
                    } else if (armMotorPower < -0.4) {
                        armMotorPower = -0.4;
                    }
                    robot.wobbleMotor.setPower(armMotorPower);

                    // Wobble Servo
                    if (gamepad1.right_bumper) {
                        robot.wobbleServo.setPosition(wobbleRightBumperPos);
                    } else if (gamepad1.left_bumper) {
                        robot.wobbleServo.setPosition(wobbleLeftBumperPos);
                    }

                    // Shooter with toggle
                    if (gamepad1.dpad_up && gamepad1.dpad_up != prevValueShooter) {
                        if (!toggleShooter){
                            robot.shooterMotor.setPower(shooterPower);
                        } else {
                            robot.shooterMotor.setPower(0);
                        }
                        toggleShooter = !toggleShooter;
                    }
                    prevValueShooter = gamepad1.dpad_up;
                    /*
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

                    */
                    // Hooper servo
                    // User input for kicker servo
                    if (gamepad1.a) {
                        isKicking = true;
                    }
                    // Controls state of isKicking based on servoCount
                    if (isKicking) {
                        servoCount += 1;
                    }
                    if (servoCount > 100) {
                        isKicking = false;
                        servoCount = 0;
                    }
                    // Kicking
                    if (isKicking) {
                        robot.hopperServo.setPosition(isKickingHopperPos);
                    }
                    else {
                        robot.hopperServo.setPosition((isNotKickingHopperPos));
                    }
                    /*
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
                    */

                    // Claw
                    if (gamepad1.dpad_left) {
                        robot.clawServo.setPosition(clawServoGrabPos);
                        robot.clawMotorDegSet(1.0, clawOverHopperPos, 7);
                        delay(0.3);
                        robot.clawServo.setPosition(clawServoReleasePos);
                        delay(0.3);
                        robot.clawMotorDegSet(1, clawOverGroundPos, 7);
                    }


                    /*
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

    public void delay(double t) { // Imitates the Arduino delay function
        runtime.reset();
        while ((runtime.seconds() < t)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

}