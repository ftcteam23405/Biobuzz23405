package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.teamcode.tuning.Globals.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp
@Configurable
@Config
public class VelocityPIDTuner extends NextFTCOpMode {

    public VelocityPIDTuner() {
        addComponents(
                BindingsComponent.INSTANCE,
                BulkReadComponent.INSTANCE
        );
    }

    public static double p = shooterP;
    public static double i = shooterI;
    public static double d = shooterD;
    public static double ff = shooterFF;

    public static double tolerance = 30;
    private static double shooterPower;

    public static PIDCoefficients coefficients = new PIDCoefficients(p, i, d);
    public static BasicFeedforwardParameters ffCoefficients = new BasicFeedforwardParameters(ff, 0, 0);
    public static int targetRPM = 0;
    public static double aveRPM = 0;
    public static int avePeriod = 5;

    ControlSystem controller = ControlSystem.builder()
            .velPid(coefficients)
            .basicFF(ffCoefficients)
            .build();

    MotorEx shooterMotorRight = new MotorEx("shooterMotorRight").brakeMode().zeroed();
    MotorEx shooterMotorLeft = new MotorEx("shooterMotorLeft").brakeMode().zeroed();




    @Override
    public void onInit() {



        controller.setGoal(new KineticState(0, 0 ,0));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry(), PanelsTelemetry.INSTANCE.getFtcTelemetry());
    }

    @Override
    public void onStartButtonPressed() {
    }

    @Override
    public void onUpdate() {
        double motorRPM = calculateRPM(shooterMotorRight.getVelocity(), 28);
        aveRPM = aveRPM * (avePeriod - 1) / avePeriod + motorRPM / avePeriod;

        controller.setGoal(new KineticState(
                0,
                targetRPM,
                0
        ));

        if (Math.abs(motorRPM - targetRPM) >= tolerance) {
            shooterPower = controller.calculate(new KineticState(0, aveRPM, 0));
        }

        shooterMotorRight.setPower(shooterPower);
        shooterMotorLeft.setPower(-shooterPower);

        telemetry.addData("Shooter Right Velocity (rpm)", motorRPM);
        telemetry.addData("Shooter Right Velocity (rpm) average", aveRPM);
        telemetry.addData("Target", targetRPM);
        telemetry.addData("Shooter Power", shooterPower);
        telemetry.update();
    }

}
