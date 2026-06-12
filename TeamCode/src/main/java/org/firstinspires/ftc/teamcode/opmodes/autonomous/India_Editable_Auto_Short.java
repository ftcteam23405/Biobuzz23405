package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tuning.PIDFMotorController;

@Autonomous
@Configurable
@Config
public class India_Editable_Auto_Short extends LinearOpMode  {

    public static double GATE_OPEN_POS = 0;
    public static double GATE_CLOSE_POS = 0;
    public static double DRIVETRAIN_POWER = 0.3;

    public static double DRIVE_TIME = 1.25;
    public static double TURN_TIME = 0.6;

    private PIDFMotorController armController;
    private DcMotor rightMotor, leftMotor;
    private Servo gateServo;

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

        gateServo = hardwareMap.get(Servo.class, "gateServo");

        // Set directions for drivetrain motors
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        gateOpen();
        forward(DRIVETRAIN_POWER, DRIVE_TIME);
        turnLeft(DRIVETRAIN_POWER, TURN_TIME);
        stopDriving();
        gateClose();
    }


    //functions for driving, can be edited
    public void backward(double power, double seconds) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        sleep((long)(seconds * 1000));
        stopDriving();

    }

    public void forward(double power, double seconds) {
        leftMotor.setPower(-power);
        rightMotor.setPower(-power);
        sleep( (long) (seconds * 1000));
        stopDriving();
    }

    public void turnRight(double power, double seconds) {
        leftMotor.setPower(power);
        rightMotor.setPower(-power);
        sleep((long)(seconds * 1000));
        stopDriving();
    }

    public void turnLeft(double power, double seconds) {
        leftMotor.setPower(-power);
        rightMotor.setPower(power);
        sleep((long)(seconds * 1000));
        stopDriving();
    }

    public void stopDriving() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void gateClose(){
        gateServo.setPosition(GATE_CLOSE_POS);
    }

    public void gateOpen(){
        gateServo.setPosition(GATE_OPEN_POS);
    }

}
