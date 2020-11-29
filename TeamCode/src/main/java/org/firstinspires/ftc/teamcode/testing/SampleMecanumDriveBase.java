package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 *<h1>SampleMecanumDriveBase</h1>
 * The SampleMecanumDriveBase class is a base class for
 * FTC hardware with PID and advanced functionality
 *<p>Autonomous and Teleop classes should use this base class</p>
 *
 * @author  Alessio Toniolo
 * @version 1.0
 * @since   2014-03-31
 */
public class SampleMecanumDriveBase {

    // Wheel attribute
    private final String wheel;

    // Motor attribute
    private final String motor;

    // Gear ratio attribute
    private final double gearRatio;

    /**
     * The encoder counts per motor revolution
     */
    public double countsPerMotorRev;

    /**
     * The diameter of the wheel in inches
     */
    public double wheelDiameter;

    /**
     * The encoder counts for every inch moved
     */
    public double countsPerInch = countsPerMotorRev / (wheelDiameter * Math.PI);

    /**
     * The encoder counts
     */
    public double countsPerDegree = countsPerMotorRev / 360;

    /**
     * The left front motor of the robot
     */
    public DcMotor lf;

    /**
     * The right front motor of the robot
     */
    public DcMotor rf;

    /**
     * The left rear motor of the robot
     */
    public DcMotor lr;

    /**
     * The right rear motor of the robot
     */
    public DcMotor rr;

    /**
     * Constructor
     * @param wheel The brand of mecanum wheel that is on your robot (lowercase) ex: rev
     * @param motor The specific motor model that is on your robot ex: hdhex
     * @param motorGearbox The gear ratio of the gearbox on your drive motors simplified ex: 20 (which would be 20:1)
     * <h2>NOTE THIS CLASS ASSUMES YOU HAVE A 1:1 GEAR RATIO FOR DRIVING</h2>
     */
    public SampleMecanumDriveBase(String wheel, String motor, double motorGearbox, double gearRatio) {
         this.wheel = wheel;
         this.motor = motor;
         this.gearRatio = gearRatio;

         if (wheel.equals("rev")) {
             wheelDiameter = 2.95275591;
         } else if (wheel.equals("tetrix")) {
             wheelDiameter = 3.858268;
         } else {
             // Default if incorrect argument passed
             wheelDiameter = 3;
         }

         if (motor.equals("hdhex")) {
             if (motorGearbox == 20) {
                 countsPerMotorRev = 560;
             } else if (motorGearbox == 40) {
                 countsPerMotorRev = 1120;
             } else {
                 // Default if incorrect argument passed
                 countsPerMotorRev = 1000;
             }
         } else if (motor.equals("torquenado")) {
             countsPerMotorRev = 1440;
         } else {
             // Default if incorrect argument passed
             countsPerMotorRev = 1440;
         }
    }

    // Hardware map
    HardwareMap hardware;

    // Time for autonomous functions
    public ElapsedTime time = new ElapsedTime();

    // PID Coefficients
    public PIDCoefficients testPID = new PIDCoefficients(0,0,0);

    /** initialize
     * <h2>REQUIRED FOR ROBOT</h2>
     * Initializes drive hardware
     * @param runUsingEncoders boolean if you are run using encoders
     * @param PID boolean if you are using PID
     * <h2>IF USING PID, SET RUN USING ENCODERS TO TRUE</h2>
     * <h2>IF USING PID, SET PID TO TRUE</h2>
     */
    public void initialize(boolean runUsingEncoders, boolean PID) {
        lf = hardware.dcMotor.get("leftfront");
        rf = hardware.dcMotor.get("rightfront");
        lr = hardware.dcMotor.get("leftrear");
        rr = hardware.dcMotor.get("rightrear");

        if (runUsingEncoders) {
            lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (PID) {
            FtcDashboard dashboard;
            dashboard = FtcDashboard.getInstance();
        }
    }

    /** forward
     * @param inches The amount of inches to move forward
     * Drives the robot forward with PID
     */
    public void forward(double inches) {
        double integral = 0;
        double repetitions = 0;

        double targetPosition = countsPerInch * inches;

        double error = lf.getCurrentPosition();
        double lastError = 0;

        while (Math.abs(error) <= 9 /*Modify*/ && repetitions < 40 /*Modify*/) {
            error = lf.getCurrentPosition() - targetPosition;
            double changeInError = lastError - error;
            integral += changeInError * time.time();
            double derivative = changeInError / time.time();
            double P = testPID.p * error;
            double I = testPID.i * integral;
            double D = testPID.d * derivative;
            lf.setPower(P + I + D);
            rf.setPower(P + I + D);
            lr.setPower(P + I + D);
            rr.setPower(P + I + D);
            error = lastError;
            time.reset();
            repetitions ++;
        }
    }

    /** backward
     * @param inches The amount of inches to move backward
     * Drives the robot backward with PID
     */
    public void backward(double inches) {
        double integral = 0;
        double repetitions = 0;

        double targetPosition = countsPerInch * inches;

        double error = lf.getCurrentPosition();
        double lastError = 0;

        while (Math.abs(error) <= 9 /*Modify*/ && repetitions < 40 /*Modify*/) {
            error = lf.getCurrentPosition() - targetPosition;
            double changeInError = lastError - error;
            integral += changeInError * time.time();
            double derivative = changeInError / time.time();
            double P = testPID.p * error;
            double I = testPID.i * integral;
            double D = testPID.d * derivative;
            lf.setPower(-(P + I + D));
            rf.setPower(-(P + I + D));
            lr.setPower(-(P + I + D));
            rr.setPower(-(P + I + D));
            error = lastError;
            time.reset();
            repetitions ++;
        }
    }

    /** strafeRight
     * @param inches The amount of inches to strafe right
     * Drives the robot to the right with PID
     */
    public void strafeRight(double inches) {
        double integral = 0;
        double repetitions = 0;

        double targetPosition = countsPerInch * inches;

        double error = lf.getCurrentPosition();
        double lastError = 0;

        while (Math.abs(error) <= 9 /*Modify*/ && repetitions < 40 /*Modify*/) {
            error = lf.getCurrentPosition() - targetPosition;
            double changeInError = lastError - error;
            integral += changeInError * time.time();
            double derivative = changeInError / time.time();
            double P = testPID.p * error;
            double I = testPID.i * integral;
            double D = testPID.d * derivative;
            lf.setPower((P + I + D));
            rf.setPower(-(P + I + D));
            lr.setPower(-(P + I + D));
            rr.setPower((P + I + D));
            error = lastError;
            time.reset();
            repetitions ++;
        }
    }

    /** strafeLeft
     * @param inches The amount of inches to strafe left
     * Drives the robot to the left with PID
     */
    public void strafeLeft(double inches) {
        double integral = 0;
        double repetitions = 0;

        double targetPosition = countsPerInch * inches;

        double error = lf.getCurrentPosition();
        double lastError = 0;

        while (Math.abs(error) <= 9 /*Modify*/ && repetitions < 40 /*Modify*/) {
            error = lf.getCurrentPosition() - targetPosition;
            double changeInError = lastError - error;
            integral += changeInError * time.time();
            double derivative = changeInError / time.time();
            double P = testPID.p * error;
            double I = testPID.i * integral;
            double D = testPID.d * derivative;
            lf.setPower(-(P + I + D));
            rf.setPower((P + I + D));
            lr.setPower((P + I + D));
            rr.setPower(-(P + I + D));
            error = lastError;
            time.reset();
            repetitions ++;
        }
    }

    /** turnRight
     * @param inches The amount of inches to turn right
     * Turns the robot right with PID
     */
    public void turnRight(double inches) {
        double integral = 0;
        double repetitions = 0;

        double targetPosition = countsPerInch * inches;

        double error = lf.getCurrentPosition();
        double lastError = 0;

        while (Math.abs(error) <= 9 /*Modify*/ && repetitions < 40 /*Modify*/) {
            error = lf.getCurrentPosition() - targetPosition;
            double changeInError = lastError - error;
            integral += changeInError * time.time();
            double derivative = changeInError / time.time();
            double P = testPID.p * error;
            double I = testPID.i * integral;
            double D = testPID.d * derivative;
            lf.setPower((P + I + D));
            rf.setPower(-(P + I + D));
            lr.setPower((P + I + D));
            rr.setPower(-(P + I + D));
            error = lastError;
            time.reset();
            repetitions ++;
        }
    }

    /** turnLeft
     * @param inches The amount of inches to turn left
     * Turns the robot left with PID
     */
    public void turnLeft(double inches) {
        double integral = 0;
        double repetitions = 0;

        double targetPosition = countsPerInch * inches;

        double error = lf.getCurrentPosition();
        double lastError = 0;

        while (Math.abs(error) <= 9 /*Modify*/ && repetitions < 40 /*Modify*/) {
            error = lf.getCurrentPosition() - targetPosition;
            double changeInError = lastError - error;
            integral += changeInError * time.time();
            double derivative = changeInError / time.time();
            double P = testPID.p * error;
            double I = testPID.i * integral;
            double D = testPID.d * derivative;
            lf.setPower(-(P + I + D));
            rf.setPower((P + I + D));
            lr.setPower(-(P + I + D));
            rr.setPower((P + I + D));
            error = lastError;
            time.reset();
            repetitions ++;
        }
    }
}
