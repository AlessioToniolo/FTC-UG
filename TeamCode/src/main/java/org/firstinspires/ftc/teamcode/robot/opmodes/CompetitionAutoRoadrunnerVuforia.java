package org.firstinspires.ftc.teamcode.robot.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.utility.BaseRobot;
import org.firstinspires.ftc.teamcode.robot.utility.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.utility.RobotAutomation;

import com.qualcomm.robotcore.util.ElapsedTime;

// AUTONOMOUS IS FOR RED SIDE, RIGHT LINE
@Autonomous(group = "Competition")
public class CompetitionAutoRoadrunnerVuforia extends LinearOpMode{

    // Vuforia fields
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY = "AX+Q8Jf/////AAABmRrI10CGCk8Cmrs+E/KkLwpA+9nReYzQgtbm10Br71odVA2bN/MI+xPlgSlkAVvHZiNEA0vgr8zVvFcR7yCfDNU/kh6jRGxN6XRvXjAyDZ0s2ixdzk7ve/GgzsIkv2zIGuT9lT9HkonMoxNGcxZqEel5NKpU0zoIfn42R121K25SH7BBfVdBdyScWwqeS5C3IfXu3z4I6+gjzxdLs/v1NKhYjokLEOcgWtoPodRbwEb32KE/RPQGv62nmW57q3voGuXFyvKCrWvH2MZ9YBHXNdpzrsDIGWnv7p1Vbrv8D62c9o0KtuK9T7NX54MkTWtY8tY1aCOyg60uQx2D4Y7Anoz5i17BvLALcCebO6puOI9u";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    // Robot fields
    BaseRobot robot = new BaseRobot();
    private ElapsedTime runtime = new ElapsedTime();
    double clawOffset  = 0.0 ;
    final double    CLAW_SPEED  = 0.02 ;

    // Automation field
    RobotAutomation automation = new RobotAutomation();

    // Roadrunner drive field for full file access
    SampleMecanumDrive drive;

    // Roadrunner start pose field for full file access
    Pose2d startPose;

    @Override
    public void runOpMode() throws InterruptedException {

        // Message to wait for initializing
        telemetry.addData("Message:", "Wait to begin autonomous: initializing");
        telemetry.update();

        // Initialize Objects
        initVuforia();
        initTfod();
        robot.initialize(hardwareMap);
        automation.initializeAutomation();

        // Make sure Vuforia is on
        if (tfod != null) {
            tfod.activate();
            // Sets zoom for camera
            tfod.setZoom(2.5, 1.78);
        }

        // Our drive base
        drive = new SampleMecanumDrive(hardwareMap);

        // We want to start the bot at the bottom right red line
        startPose = new Pose2d(-63.0, -50.0, Math.toRadians(180.0));
        drive.setPoseEstimate(startPose);

        // Message to signal robot is ready to have camera checked or run
        telemetry.addData("Message:", "Robot is ready to begin autonomous");
        telemetry.update();

        // Opmode
        waitForStart();
        if (isStopRequested()) return;

        // Vision
        String label = "None";
        if (opModeIsActive()) {
            runtime.reset();
            double t = 2; // two seconds
            while (opModeIsActive() && (runtime.seconds() < t)) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            label = recognition.getLabel();
                            telemetry.addData(String.format("label (%d)", i), label);
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                        }
                        telemetry.update();
                    }
                }
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }
        if (label.equals("None")) {
            goToA();
        } else if (label.equals("Single")) {
            goToB();
        } else if (label.equals("Quad")) {
            goToC();
        } else {
            goToC();
        }
        // Test output and detection
        telemetry.addData("Final Label", label);
        telemetry.update();
    }



    // Vuforia method for initializing
    private void initVuforia() {

        // Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine
    }

    // Tensorflow method for initializing
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    // Autonomous sequence for target a
    public void goToA() {
        // TODO: finish paths and add mechanism functionality

        // todo: add shooting
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(0.0, -60.0, Math.toRadians(-90.0)))
                .build();
        // todo: add marker for wobble with async arm turning then marker for servo to open once traj is reached
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(-31.5, -27.0, Math.toRadians(90.0)))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeLeft(5)
                .build();
        // todo: add marker for servo capture
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .strafeRight(5)
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .splineTo(new Vector2d(10.0 , -15.0), 0.0)
                .build();
        // TODO: figure out which way robot will finish orientation and field centric wise

        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
    // Autonomous sequence for target b
    public void goToB() {
        // TODO: finish paths and add mechanism functionality

        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
    // Autonomous sequence for target c
    public void goToC() {
        // TODO: finish paths and add mechanism functionality

        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = drive.getPoseEstimate();
    }

}
