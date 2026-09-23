package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.commandbase.Alliance;
import java.util.ArrayList;
import java.util.List;

@Configurable
@Config
public class Limelight {
    private Limelight3A limelight;
    private Alliance alliance;
    private static final int redTags = 0, blueTags = 1, blob = 2;
    private int tagsPipeline;

    private int currentPipeline;

    public static double distanceOffset = 8; // inches added to the camera-to-tag distance (camera to shooter)
    public static long maxStalenessMs = 150; // ignore results older than this

    public Limelight(HardwareMap hardwareMap, Alliance a) {
        alliance = a;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if (alliance == Alliance.RED) {
            tagsPipeline = redTags;
        } else {
            tagsPipeline = blueTags;
        }
        limelight.setPollRateHz(20);
        setPipleline(tagsPipeline);
        limelight.start();
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

    public void setPipleline(int pipleline) {
        limelight.pipelineSwitch(pipleline);
        currentPipeline = pipleline;
    }

    public int getPipleline() {
        return currentPipeline;
    }


    //finds distance to closest tag
    public double distanceFromTag() {
        LLResult result = tagResult();

        if (result == null) return 0;

        double closest = Double.MAX_VALUE;

        // closest visible tag; the pipeline only detects our alliance's tags
        for (LLResultTypes.FiducialResult i: result.getFiducialResults()) {
            if (i != null)
                closest = Math.min(closest, distanceTo(i));
        }

        return closest == Double.MAX_VALUE ? 0 : closest;
    }

    // horizontal distance from camera to tag, in inches
    private double distanceTo(LLResultTypes.FiducialResult tag) {
        Position p = tag.getCameraPoseTargetSpace().getPosition().toUnit(DistanceUnit.INCH);

        // x is right/left of the tag and z is out from the tag, so this ignores height
        return Math.hypot(p.x, p.z) + distanceOffset;
    }

    public double angle() {
        LLResult result = tagResult();

        if (result == null) return 0;

        return result.getTx(); // horizontal offset from crosshair to primary detection, in degrees
    }

    // latest result from the tags pipeline, or null if there's no fresh detection
    private LLResult tagResult() {
        switchToShootPipeline();
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) return null;
        if (result.getStaleness() > maxStalenessMs) return null;
        if (result.getPipelineIndex() != currentPipeline) return null; // still switching pipelines

        return result;
    }

    public void switchToShootPipeline() {
        if (currentPipeline != tagsPipeline)
            setPipleline(tagsPipeline);
        if (!limelight.isRunning())
            limelight.start();
    }

    public List<Integer> visibleTagIds() {
        List<Integer> ids = new ArrayList<>();
        LLResult result = tagResult();

        if (result == null) return ids;

        for (LLResultTypes.FiducialResult i: result.getFiducialResults()) {
            if (i != null)
                ids.add(i.getFiducialId());
        }
        return ids;
    }

}
