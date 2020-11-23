package org.firstinspires.ftc.teamcode.robot.utility;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class RobotAutomation {

    // Private robot
    private BaseRobot robot = new BaseRobot();
    // Private runtime
    private ElapsedTime runtime = new ElapsedTime();

    // Constructor
    public RobotAutomation() {}

    // Does hopper servo movement
    private void hopperFlick() {
        // TODO: APPLY TODOS FOR SERVOS FROM singleShoot()
        robot.hopperServo.setPosition(1);
        robot.hopperServo.setPosition(0);
    }

    // Shoots a single ring from hopper
    // TODO: not in use due to unfinished robot
    public void singleShoot(double duration) {
        // TODO: check if power is negative or positive for shooter
        robot.shooterMotor.setPower(-1);
        delay(duration);
        // TODO: tune servo position
        robot.hopperServo.setPosition(1);
        robot.shooterMotor.setPower(0);
        robot.hopperServo.setPosition(0);
    }

    // Shoots all rings from hopper
    public void multipleShoot(double duration) {
        // TODO: APPLY TODOS FROM singleShoot()
        robot.shooterMotor.setPower(-1);
        delay(duration);
        hopperFlick();
        hopperFlick();
        hopperFlick();
        robot.shooterMotor.setPower(0);
    }

    // Arduino delay function
    private void delay(double t) {
        runtime.reset();
        while ((runtime.seconds() < t)) {}
    }

    // Initializes private robot class
    public void initializeAutomation() {
        robot.initialize(hardwareMap);
    }
}
