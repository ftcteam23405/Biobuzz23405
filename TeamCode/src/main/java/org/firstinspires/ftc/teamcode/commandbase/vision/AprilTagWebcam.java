package org.firstinspires.ftc.teamcode.commandbase.vision;

import android.util.Size;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagWebcam {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();
    private TelemetryManager telemetryM;

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    public void initalize(HardwareMap hwMap, TelemetryManager telemetryM) {
        this.telemetryM = telemetryM;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    public void update() {
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public void displayDetectionTelemetry(AprilTagDetection detectedId) {
        if (detectedId == null) {
            return;
        }

        if (detectedId.metadata != null) {
            telemetryM.debug(String.format("\n==== (ID %d) %s", detectedId.id, detectedId.metadata.name));
            telemetryM.debug(String.format("XYZ %6.1f %6.1f %6.1f  (cm)", detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));
            telemetryM.debug(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
            telemetryM.debug(String.format("RBE %6.1f %6.1f %6.1f  (cm, deg, deg)", detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));
        } else {
            telemetryM.debug(String.format("\n==== (ID %d) Unknown", detectedId.id));
            telemetryM.debug(String.format("Center %6.0f %6.0f   (pixels)", detectedId.center.x, detectedId.center.y));
        }
    }

    public AprilTagDetection getTagBySpecificID(int id){
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    public AprilTagDetection getFirstDetectedTag() {
        if (detectedTags != null && !detectedTags.isEmpty()) {
            return detectedTags.get(0); // return the first detected tag
        }
        return null; // no tags detected
    }

    public double getTagBearing(int tagID) {
        AprilTagDetection tag = getTagBySpecificID(tagID);
        return (tag != null) ? tag.ftcPose.bearing : Double.NaN;
    }

    public double getTagDistance(int tagID) {
        AprilTagDetection tag = getTagBySpecificID(tagID);
        return (tag != null) ? tag.ftcPose.range : Double.NaN;
    }

    public double getTagDistance(AprilTagDetection detection) {
        return detection.ftcPose.y;
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    public void start() {
        if (visionPortal != null && visionPortal.getCameraState() == VisionPortal.CameraState.STOPPING_STREAM) {
            visionPortal.resumeStreaming();
        }
    }

    public void pause() {
        if (visionPortal != null && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopStreaming();
        }
    }

    // Normalize an angle to [-180, 180)
    private double normalizeAngleDegrees(double ang) {
        // normalize error to [-180, 180]
        double a = ang % 360.0;
        if (a >= 180.0) a -= 360.0;
        if (a < -180.0) a += 360.0;
        return a;
    }
}
