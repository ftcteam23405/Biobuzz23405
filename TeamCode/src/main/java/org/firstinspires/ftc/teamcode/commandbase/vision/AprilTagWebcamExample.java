package org.firstinspires.ftc.teamcode.commandbase.vision;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class AprilTagWebcamExample extends OpMode {

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    private TelemetryManager telemetryM;

    @Override
    public void init() {
        aprilTagWebcam.initalize(hardwareMap, telemetryM);
    }

    @Override
    public void loop() {
        //update vision portal
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificID(20);
        AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificID(24);
        aprilTagWebcam.displayDetectionTelemetry(id20);
        aprilTagWebcam.displayDetectionTelemetry(id24);
    }

    @Override
    public void stop() {
        aprilTagWebcam.stop();
    }
}
