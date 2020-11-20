package org.firstinspires.ftc.teamcode.robot.utility;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BaseRobot {

    // Mechanism Motors and Servos
    public DcMotor wobbleArm = null;
    public DcMotor testMotor = null;
    public DcMotor surgicalTubingIntake = null;
    public DcMotor wheelIntake = null;
    public Servo rightHand    = null;
    public Servo leftHand   = null;

    // Constants for Arm and Servo Operation
    public static final double MID_SERVO        =  0.5;
    public static final double ARM_UP_POWER     =  0.45;
    public static final double ARM_DOWN_POWER   = -0.45;

    // For Encoder Functions
    private double     COUNTS_PER_MOTOR_REV          = 1440 ;    // eg: TETRIX Motor Encoder
    private final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private double     WHEEL_DIAMETER_INCHES         = 2.95275591 ;     // For figuring circumference
    private double     COUNTS_PER_INCH               = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private double COUNTS_PER_DEGREE                 = COUNTS_PER_MOTOR_REV / 360;
    private double     DRIVE_SPEED                   = 1.0;
    private double     TURN_SPEED                    = 1.0;

    // Local OpMode members
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    // Constructor
    public BaseRobot() {

    }

    public void initialize(HardwareMap ahwMap){
        // Save reference to Hardware map
        hwMap = ahwMap;

        wobbleArm = hwMap.dcMotor.get("wobblearm");
        testMotor = hwMap.dcMotor.get("testmotor");
        surgicalTubingIntake = hwMap.dcMotor.get("surgicalTubingIntake");

        wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        surgicalTubingIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftHand = hwMap.servo.get("lefthand");
    }

    public void wobbleArmDegSet(double speed, double deg, double timeoutS){
        int target;

        deg = deg * 2;
        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (Math.abs(speed) > DRIVE_SPEED) {
            speed = DRIVE_SPEED;
        }

        target = (int)(deg * COUNTS_PER_DEGREE);
        wobbleArm.setTargetPosition(target);
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        period.reset();
        wobbleArm.setPower(Math.abs(speed));
    }

    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
