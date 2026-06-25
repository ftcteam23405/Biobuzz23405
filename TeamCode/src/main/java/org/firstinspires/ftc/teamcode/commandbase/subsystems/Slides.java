package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@Configurable
public class Slides {
    private double error = 0, power = 0;
    private DcMotorEx rightSlideMotor, leftSlideMotor;

    private double target = 0;
    private PIDFController fastController, slowController; // PIDFController for slides

    public static double pidfSwitch = 30; // target tolerance for turret

    public static double kP = 0.01, kD = 0.00039, kF = 0;
    public static double sP = .005, sD = 0.0001, sF = 0;

    public static double upPos = 1200;
    public static double downPos = 0;

    public Slides(HardwareMap hardwareMap) {
        leftSlideMotor = hardwareMap.get(DcMotorEx.class, "sl");
        rightSlideMotor = hardwareMap.get(DcMotorEx.class, "sr");

        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlideMotor.setPower(0);
        rightSlideMotor.setPower(0);

        target = 0;

        fastController = new PIDFController(new PIDFCoefficients(kP, 0, kD, kF));
        slowController = new PIDFController(new PIDFCoefficients(sP, 0, sD, sF));
    }

    public void periodic() {
        fastController.setCoefficients(new PIDFCoefficients(kP, 0, kD, kF));
        slowController.setCoefficients(new PIDFCoefficients(sP, 0, sD, sF));
        error = getTarget() - getRightPosition();
        if (Math.abs(error) > pidfSwitch) {
            fastController.updateError(error);
            fastController.updateFeedForwardInput(Math.signum(error));
            power = fastController.run();
        } else {
            slowController.updateError(error);
            power = slowController.run();
        }
        rightSlideMotor.setPower(power);
        leftSlideMotor.setPower(power);
    }



    public CommandBuilder up() {
        return Commands.instant(() -> setTarget(upPos));
    }

    public CommandBuilder down() {
        return Commands.instant(() -> setTarget(downPos));
    }

    public CommandBuilder reset() {
        return Commands.instant(this::resetSlides);
    }


    public void resetSlides() {
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setTarget(0);
    }

    public double getTarget(){
        return target;
    }

    public void setTarget(double ticks) {
        target = ticks;
    }

    public double getError() {
        return error;
    }

    public boolean isReady() {
        return Math.abs(getError()) < pidfSwitch;
    }

    public double getLeftPosition() {
        return leftSlideMotor.getCurrentPosition();
    }

    public double getRightPosition() {
        return rightSlideMotor.getCurrentPosition();
    }

    public double getVelocity() {
        return rightSlideMotor.getVelocity();
    }

    public String getLeftCurrent() {
        return "Left Slide Motor: " + leftSlideMotor.getCurrent(CurrentUnit.AMPS);
    }

    public String getRightCurrent() {
        return "Right Slide Motor: " + rightSlideMotor.getCurrent(CurrentUnit.AMPS);
    }
}
