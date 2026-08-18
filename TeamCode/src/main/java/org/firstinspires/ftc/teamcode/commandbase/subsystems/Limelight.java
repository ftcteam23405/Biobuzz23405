package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commandbase.Alliance;
import java.util.List;

@Configurable
@Config
public class Limelight {
    private Limelight3A limelight;
    private Alliance alliance;
    private static final int shoot = 0, zone = 1;
    private int pipeline = shoot;

    public Limelight(HardwareMap hardwareMap, Alliance a) {
        alliance = a;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        switchToShoot();
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public void pause() {
        limelight.pause();
    }

    public double distanceFromTag(double tagID) {
        switchToShoot();
        List<LLResultTypes.FiducialResult> r = limelight.getLatestResult().getFiducialResults();

        if (r.isEmpty()) return 0;

        LLResultTypes.FiducialResult target = null;
        for (LLResultTypes.FiducialResult i: r) {
            if (i != null && i.getFiducialId() ==  tagID) {
                target = i;
                break;
            }
        }

        if (target != null) {
            double x = (target.getCameraPoseTargetSpace().getPosition().x / DistanceUnit.mPerInch) + 8; // right/left from tag
            double z = (target.getCameraPoseTargetSpace().getPosition().z / DistanceUnit.mPerInch) + 8; // forward/back from tag

            Vector e = new Vector();
            e.setOrthogonalComponents(x, z);
            return e.getMagnitude();
        }

        return 0;
    }

    public double distanceFromBlue() {
        return distanceFromTag(20);
    }

    public double distanceFromRed() {
        return distanceFromTag(24);
    }

    public double angleFromTag(double tagID) {
        switchToShoot();
        List<LLResultTypes.FiducialResult> r = limelight.getLatestResult().getFiducialResults();

        if (r.isEmpty()) return 0;

        LLResultTypes.FiducialResult target = null;
        for (LLResultTypes.FiducialResult i: r) {
            if (i != null && i.getFiducialId() ==  tagID) {
                target = i;
                break;
            }
        }

        if (target != null)
            return target.getTargetXDegrees();

        return 0;
    }

    public double angleFromBlue() {
        return angleFromTag(20);
    }

    public double angleFromRed() {
        return angleFromTag(24);
    }

    public double angleFromShoot() {
        return alliance == Alliance.BLUE ? angleFromBlue() : angleFromRed();
    }

    public double distanceFromShoot() {
        return alliance == Alliance.BLUE ? distanceFromBlue() : distanceFromRed();
    }

    public void switchToShoot() {
        if (pipeline != shoot)
            limelight.pipelineSwitch(shoot);
        limelight.setPollRateHz(20);
        limelight.start();
    }
}