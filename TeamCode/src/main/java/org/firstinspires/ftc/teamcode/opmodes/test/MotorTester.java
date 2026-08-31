package org.firstinspires.ftc.teamcode.opmodes.test;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MotorTester extends LinearOpMode {

    public static double speed;
    public void runOpMode() throws InterruptedException {
        final DcMotorEx motor;
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        waitForStart();

        speed = -gamepad1.left_stick_y;

        while(opModeIsActive()){
            motor.setPower(speed);
        }
    }
}
