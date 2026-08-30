package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp
public class ServoTester extends LinearOpMode {
    public static double armPos = 0.5 ;
    public static double latchPos = 0.5;
    public void runOpMode() throws InterruptedException {
        final Servo rightArmServo, leftArmServo, latchServo;
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        latchServo = hardwareMap.get(Servo.class, "latchServo");
        waitForStart();


        while(opModeIsActive()){
            latchServo.setPosition(latchPos);
            rightArmServo.setPosition(1 - armPos);
            leftArmServo.setPosition(armPos);
        }
    }

}
