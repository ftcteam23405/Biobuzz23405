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
import org.firstinspires.ftc.teamcode.commandbase.vision.PollenDetector;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@Configurable
@Config
public class Limelight {
    private Limelight3A limelight;
    private Alliance alliance;
    private static final int redTags = 0, blueTags = 1, blob = 2;
    private int tagsPipeline;

    private int currentPipeline;
    private PollenDetector pollenDetector;

    // blob results, refreshed in periodic()
    private List<PollenDetector.Ball> balls = new ArrayList<>();
    private int pollenCount = 0;
    private PollenDetector.Ball bestPollen = null;

    public static double distanceOffset = 8; // inches added to the camera-to-tag distance (camera to shooter)
    public static long maxStalenessMs = 150; // ignore results older than this

    public Limelight(HardwareMap hardwareMap, Alliance a) {
        alliance = a;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        pollenDetector = new PollenDetector(hardwareMap, "limelight");
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
    
    // straight-line distance to the closest goal in inches, or NaN if no tag is visible
    // the pipeline only detects our alliance's tags, so the closest tag is on the closest goal
    public double distanceToGoal() {
        LLResult result = tagResult();

        if (result == null) return Double.NaN;

        double closest = Double.NaN;
        for (LLResultTypes.FiducialResult i: result.getFiducialResults()) {
            if (i == null) continue;
            double d = distanceTo(i);
            if (Double.isNaN(closest) || d < closest)
                closest = d;
        }

        return closest;
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

        return result.getTx(); // horizontal offset from crosshair to primary detection, deg
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

    public void switchToBlobPipeline() {
        if (currentPipeline != blob)
            setPipleline(blob);
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

    // distance to every visible tag, keyed by tag id
    public Map<Integer, Double> tagDistances() {
        Map<Integer, Double> distances = new LinkedHashMap<>();
        LLResult result = tagResult();

        if (result == null) return distances;

        for (LLResultTypes.FiducialResult i: result.getFiducialResults()) {
            if (i != null)
                distances.put(i.getFiducialId(), distanceTo(i));
        }
        return distances;
    }

    // call once per loop; only does work while on the blob pipeline
    public void periodic() {
        if (currentPipeline != blob) {
            clearBlobs();
            return;
        }

        LLResult result = limelight.getLatestResult();

        // still switching pipelines, so the python output is from the old pipeline
        if (result == null || currentPipeline != blob) {
            clearBlobs();
            return;
        }

        balls = pollenDetector.update();
        pollenCount = pollenDetector.getPollenCount();
        bestPollen = pollenDetector.getBestPollen();
    }

    private void clearBlobs() {
        balls = new ArrayList<>();
        pollenCount = 0;
        bestPollen = null;
    }

    // pollen visible, median over the last few frames (use this for decisions)
    public int getPollenCount() {
        return pollenCount;
    }

    // closest pollen (largest radius), or null if none
    public PollenDetector.Ball getBestPollen() {
        return bestPollen;
    }

    public boolean hasPollen() {
        return bestPollen != null;
    }

    // every ball from the latest frame, pollen and other
    public List<PollenDetector.Ball> getBalls() {
        return balls;
    }

    public boolean hasFreshBlobData() {
        return currentPipeline == blob && pollenDetector.hasFreshData();
    }

}
