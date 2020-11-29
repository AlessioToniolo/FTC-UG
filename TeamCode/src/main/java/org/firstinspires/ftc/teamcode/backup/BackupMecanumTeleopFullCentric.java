package org.firstinspires.ftc.teamcode.backup;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// Motor power variable names are different from BackupMecanumTeleopFull.java for time reasons
@TeleOp(group = "Backup")
public class BackupMecanumTeleopFullCentric extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // IMU Fields
        BNO055IMU imu;
        Orientation angles;
        Acceleration gravity;
        BNO055IMU.Parameters imuParameters;

        // Hardware map
        HardwareMap hwMap = null;

        // Drive motors
        DcMotor leftFront = hwMap.dcMotor.get("leftfront");
        DcMotor rightFront = hwMap.dcMotor.get("rightfront");
        DcMotor leftRear = hwMap.dcMotor.get("leftrear");
        DcMotor rightRear = hwMap.dcMotor.get("rightrear");

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // IMU Initialization
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);

        // Opmode
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {

            // IMU Data
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double imuAngle = angles.firstAngle;

            // Compute field centric vector
            double theta;
            if (Math.abs(gamepad1.left_stick_x) < 0.05)
            {
                theta = Math.atan(gamepad1.left_stick_y/0.05);
            }
            else {
                if (gamepad1.left_stick_x < 0) {
                    theta = Math.atan((-1*gamepad1.left_stick_y)/(gamepad1.left_stick_x));
                    //theta = Math.atan((gamepad1.left_stick_y)/(gamepad1.left_stick_x));
                }
                else {
                    theta = Math.atan(gamepad1.left_stick_y/gamepad1.left_stick_x);
                }
            }
            double mag = Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2));
            theta = theta - Math.PI/2;
            if (gamepad1.left_stick_x > 0) {
                theta = theta * -1;
            }
            double newTheta = theta - ((imuAngle/360.0) * 2 * Math.PI);

            // Compute power for wheels
            double leftFrontPower = mag * Math.cos(newTheta + (Math.PI/4)) - gamepad1.right_stick_x;
            double rightFrontPower = mag * Math.sin(newTheta + (Math.PI/4)) + gamepad1.right_stick_x;
            double leftRearPower = mag * Math.sin(newTheta + (Math.PI/4)) - gamepad1.right_stick_x;
            double rightRearPower = mag * Math.cos(newTheta + (Math.PI/4)) + gamepad1.right_stick_x;

            // Set motor powers
            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower);
        }
    }
}
