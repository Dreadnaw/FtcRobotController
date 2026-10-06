package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Field-Oriented Mecanum TeleOp Drive for FTC.
 * 
 * Hardware Configuration Requirements in Robot Configuration:
 * - "front_left_drive"  : Front Left Drive Motor
 * - "back_left_drive"   : Back Left Drive Motor
 * - "front_right_drive" : Front Right Drive Motor
 * - "back_right_drive"  : Back Right Drive Motor
 * - "imu"               : REV Control Hub / Expansion Hub IMU
 */
@TeleOp(name = "Field-Oriented Mecanum Drive", group = "TeleOp")
public class FieldOrientedMecanumDrive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors from Hardware Map
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("m3");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("m0");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("m2");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("m1");

        // Reverse left side motors so positive power pushes the robot forward
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        // Retrieve the IMU from hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");

        /*
         * Specify the orientation of your REV Control Hub / Expansion Hub.
         * Adjust logoDirection and usbDirection to match how the hub is mounted on your robot.
         * Examples for LogoFacingDirection: UP, DOWN, FORWARD, BACKWARD, LEFT, RIGHT
         * Examples for UsbFacingDirection:  FORWARD, BACKWARD, UP, DOWN, LEFT, RIGHT
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Press OPTIONS / START or Y on gamepad1 to reset Yaw heading.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Gamepad inputs (Y-stick is inverted by default in FTC gamepads)
            double y = -gamepad1.left_stick_y;  // Push forward = positive Y
            double x = gamepad1.left_stick_x;   // Push right = positive X
            double rx = gamepad1.right_stick_x; // Push right = positive RX (clockwise rotation)

            // Reset heading to 0 when OPTIONS / START or Y is pressed
            if (gamepad1.options || gamepad1.y) {
                imu.resetYaw();
            }

            // Get robot current heading in radians
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement vector by negative robot heading
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Calculate Mecanum wheel powers
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            // Output power to motors
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Telemetry updates
            telemetry.addData("Heading (Deg)", Math.toDegrees(botHeading));
            telemetry.addData("FL Power", "%.2f", frontLeftPower);
            telemetry.addData("BL Power", "%.2f", backLeftPower);
            telemetry.addData("FR Power", "%.2f", frontRightPower);
            telemetry.addData("BR Power", "%.2f", backRightPower);
            telemetry.update();
        }
    }
}
