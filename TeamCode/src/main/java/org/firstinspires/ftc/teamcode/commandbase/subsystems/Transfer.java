package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Transfer {

    public static double speed = 0.9;

    private CRServo transferServo;

    public Transfer(HardwareMap hardwareMap) {
        transferServo = hardwareMap.get(CRServo.class, "transferServo");
    }

    public void transferOn() {
        transferServo.setPower(speed);
    }
    public void transferOff() {
        transferServo.setPower(0);
    }

    public CommandBuilder on() {
        return Commands.instant(this::transferOn);
    }
    public CommandBuilder off() {
        return Commands.instant(this::transferOff);
    }

}
