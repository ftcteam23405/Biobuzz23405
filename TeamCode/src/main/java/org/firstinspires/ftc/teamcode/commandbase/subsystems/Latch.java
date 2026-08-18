package org.firstinspires.ftc.teamcode.commandbase.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@Config
public class Latch {
    private Servo latchServo;
    public static double downPos = 0.52;
    public static double upPos = 0.3;

    public Latch(HardwareMap hardwareMap){
        latchServo = hardwareMap.get(Servo.class, "latchServo");
    }

    public void latchDown() {
        latchServo.setPosition(downPos);
    }

    public void latchUp() {
        latchServo.setPosition(upPos);
    }

    public CommandBuilder up() { return Commands.instant(this::latchUp); }
    public CommandBuilder down() { return Commands.instant(this::latchDown); }

    public boolean isDown() {
        return latchServo.getPosition() == downPos;
    }
}
