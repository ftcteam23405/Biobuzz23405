package org.firstinspires.ftc.teamcode.tuning;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp
public class AprilTagLimelightTest extends NextFTCOpMode {

    public AprilTagLimelightTest() {
        addComponents(
                new PedroComponent(Constants::createFollower)
        );
    }

    private Limelight3A limelight;

    private double distance;


    @Override
    public void onInit() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        PedroComponent.follower().setStartingPose(new Pose(0,0,0).withHeading(Math.toRadians(0)));
    }

    @Override
    public void onStartButtonPressed() {
        limelight.start();
    }

    public void onUpdate() {
        limelight.updateRobotOrientation(PedroComponent.follower().getHeading());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
            distance = getDistanceFromTag(llResult.getTa());
            telemetry.addData("Distance", distance);
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("BotPose", botPose.toString());
            telemetry.addData("Yaw", botPose.getOrientation().getYaw());
            telemetry.update();
        }
    }

    public double getDistanceFromTag(double ta) {
        double scale = 0; //change this with testing
        double distance = (scale / ta);
        return distance;
    }
}