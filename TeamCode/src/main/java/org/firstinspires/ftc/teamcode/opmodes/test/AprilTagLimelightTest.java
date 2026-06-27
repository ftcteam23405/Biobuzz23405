package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class AprilTagLimelightTest extends OpMode {


    private Follower follower;
    private Limelight3A limelight;

    private double distance;


    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        follower.setStartingPose(new Pose(0,0,0).withHeading(Math.toRadians(0)));
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
    }

    @Override
    public void start() {
        limelight.start();
    }

    public void loop() {
        follower.update();
        limelight.updateRobotOrientation(follower.getHeading());
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