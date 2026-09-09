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

    public Servo leftArmServo;
    public Servo rightArmServo;
    private static final double depositPos = 0;
    private static final double intakePos = 0.62;

    public SlideArm(HardwareMap hardwareMap) {
        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
    }

    public void rightToPosition(double pos) {
        rightArmServo.setPosition(1 - pos);
    }

    public Command leftToPosition(double pos) {
        return Command.build()
                .setStart(() -> leftArmServo.setPosition(pos))
                .requiring(this);
    }
    public void toPosition(double allPos) {
        rightToPosition(allPos);
        leftToPosition(allPos);
    }

    public void toDeposit() {
        toPosition(depositPos);
    }

    public void toIntake() {
        toPosition(intakePos);
    }

}
