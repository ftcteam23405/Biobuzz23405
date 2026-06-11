package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.tuning.PIDFMotorController;

@Autonomous
@Configurable
@Config

public class India_Editable_Auto extends LinearOpMode {

    public static double MAX_ARM_POWER = 0.7;
    public static int ARM_INITIAL_ANGLE = 90; //deg
    public static int ARM_INTAKE_POSITION = 2250;
    public static int ARM_UP_POSITION = 0;
    public static double INTAKE_POWER = 0.75;
    public static double OUTTAKE_POWER = 1;
    public static int OUTTAKE_POS = 500;
    public static double DRIVETRAIN_POWER = 0.75;
    private PIDFMotorController armController;
    private DcMotor rightMotor, leftMotor;
    private com.qualcomm.robotcore.hardware.CRServo intakeServo;

    private Thread pidThread;
    private volatile boolean pidThreadRunning = true;

    private void runPIDIterations() {
        PIDFMotorController.MotorData armMotorData = armController.runIteration();
        telemetry.addData("Arm Position", armMotorData.CurrentPosition);
        telemetry.addData("Arm Target", armMotorData.TargetPosition);
        telemetry.addData("Arm Power", armMotorData.SetPower);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor = hardwareMap.dcMotor.get("leftMotor");

        DcMotorEx intakeArmMotor = hardwareMap.get(DcMotorEx.class, "intakeArmMotor");

        intakeServo = hardwareMap.get(CRServo .class, "intakeServo");

        final double armTicksInDegrees = 537.7 / 360.0;

        // Initialize PIDF controllers for the arm
        armController = new PIDFMotorController(intakeArmMotor, 0.01, 0.2, 0.0005, 0.4, armTicksInDegrees, MAX_ARM_POWER, ARM_INITIAL_ANGLE);

        // Set directions for drivetrain motors
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        pidThread = new Thread(() -> {
            while (pidThreadRunning && !isStopRequested()) {
                runPIDIterations();
                telemetry.update();
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        });

        pidThread.start();
        waitForStart();

        armController.setTargetPosition(ARM_UP_POSITION);
        forward(DRIVETRAIN_POWER,1);
        turnLeft(DRIVETRAIN_POWER,1);
        stopDriving();
        outtakePreload();

        pidThreadRunning = false;
        pidThread.join();
    }


    //functions for driving, can be edited
    public void backward(double power,long seconds) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        sleep((seconds * 1000));
        stopDriving();

    }

    public void forward(double power, long seconds) {
        leftMotor.setPower(-power);
        rightMotor.setPower(-power);
        sleep((seconds * 1000));
        stopDriving();
    }

    public void turnRight(double power, long seconds) {
        leftMotor.setPower(power);
        rightMotor.setPower(-power);
        sleep((seconds * 1000));
        stopDriving();
    }

    public void turnLeft(double power, long seconds) {
        leftMotor.setPower(-power);
        rightMotor.setPower(power);
        sleep((seconds * 1000));
        stopDriving();
    }

    public void stopDriving() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void outtakePreload(){
        armController.setTargetPosition(OUTTAKE_POS);
        sleep(1000);
        intakeServo.setPower(OUTTAKE_POWER);
        sleep(2000);
        intakeServo.setPower(0);
    }
}
