package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp
public class ServoTester extends LinearOpMode {
    public static double WRIST_SERVO_POSITION = 0;
    public void runOpMode() throws InterruptedException {
        final Servo gateServo;
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        waitForStart();

        while(opModeIsActive()){
            gateServo.setPosition(WRIST_SERVO_POSITION);
        }
    }

}
