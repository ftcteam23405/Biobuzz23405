package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static com.pedropathing.ivy.groups.Groups.*;

@Configurable
@Config
public class SlideArm {

    private Servo leftArmServo;
    private Servo rightArmServo;
    public static double depositPos = 0.8;
    public static double intakePos = 0.2;

    public SlideArm(HardwareMap hardwareMap) {
        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
    }

    public Command rightToPosition(double pos) {
        return Command.build()
                .setStart(() -> rightArmServo.setPosition(1 - pos))
                .requiring(this);
    }

    public Command leftToPosition(double pos) {
        return Command.build()
                .setStart(() -> leftArmServo.setPosition(pos))
                .requiring(this);
    }
    public Command toPosition(double allPos) {
        return parallel(rightToPosition(allPos), leftToPosition(allPos));
    }

    public Command toDeposit() {
        return toPosition(depositPos);
    }

    public Command toIntake() {
        return toPosition(intakePos);
    }

}
