package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.tuning.PIDFMotorController;

/**
 * TeleOp mode for controlling the robot. Integrates driving, arm, slide, and intake systems.
 * Uses PIDF controllers to manage motor movements for precise positioning.
 */
@Config
@Configurable
@TeleOp
public class FAST_BOT_PR extends LinearOpMode {

    public static double MAX_ARM_POWER = 0.7;
    public static int ARM_INITIAL_ANGLE = 90; //deg
    public static int ARM_INTAKE_POSITION = 2250;
    public static int ARM_UP_POSITION = 0;
    public static double INTAKE_POWER = -0.75;
    public static double OUTTAKE_POWER = 1;

    public static int OUTTAKE_POS = 500;
    public static double DRIVETRAIN_POWER = 0.2;
    private PIDFMotorController armController;
    private DcMotor rightMotor, leftMotor;
    private CRServo intakeServo;
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        initializeHardware();

        // Reset IMU orientation
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        imu.initialize(imuParameters);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // Wait for start
        telemetry.addLine("Initialized. Ready to start.");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            handleDriving();
            intakeControl();
            runPIDIterations();
            telemetry.update();
        }
    }

    /**
     * Initializes the hardware components and PIDF controllers.
     */
    private void initializeHardware() {
        // Initialize motors and servos
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor = hardwareMap.dcMotor.get("leftMotor");

        DcMotorEx intakeArmMotor = hardwareMap.get(DcMotorEx.class, "intakeArmMotor");

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        final double armTicksInDegrees = 537.7 / 360.0;

        // Initialize PIDF controllers for the arm and slide
        armController = new PIDFMotorController(intakeArmMotor, 0.01, 0.25, 0.001, 0.4, armTicksInDegrees, MAX_ARM_POWER, ARM_INITIAL_ANGLE);

        // Set directions for drivetrain motors
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");
    }

    private void handleDriving() {

        // Set motor powers
        double y = (gamepad1.left_stick_y); // Remember, Y stick is reversed!
        double rx = (gamepad1.left_stick_x);

        leftMotor.setPower(y + rx);
        rightMotor.setPower(y - rx);
    }



    private void intakeControl(){

        double arm_add = gamepad1.right_trigger;
        double arm_subtract = gamepad1.left_trigger;

        if (gamepad1.a){
            armController.setTargetPosition(ARM_INTAKE_POSITION);
        } else if (gamepad1.y){
            armController.setTargetPosition(ARM_UP_POSITION);
        } else if (gamepad1.b){
            intakeServo.setPower(0);
        } else if (gamepad1.right_bumper){
            intakeServo.setPower(INTAKE_POWER);
        } else if (gamepad1.left_bumper){
            intakeServo.setPower(OUTTAKE_POWER);
        } else if (gamepad1.x){
            armController.setTargetPosition(OUTTAKE_POS);
        } else if (gamepad1.right_trigger > 0.1) {
            armController.setTargetPosition(armController.getCurrentPosition() + (int) (arm_add * 100));
        } else if (gamepad1.left_trigger > 0.1){
            armController.setTargetPosition(armController.getCurrentPosition() - (int) (arm_subtract * 100));
        }
    }

    /**
     * Controls the bucket position based on gamepad2 inputs.
     */

    private void runPIDIterations() {
        PIDFMotorController.MotorData armMotorData = armController.runIteration();
        telemetry.addData("Arm Position", armMotorData.CurrentPosition);
        telemetry.addData("Arm Target", armMotorData.TargetPosition);
        telemetry.addData("Arm Power", armMotorData.SetPower);
    }
}