package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.commandbase.Robot.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierPoint;

import org.firstinspires.ftc.teamcode.commandbase.Alliance;
import org.firstinspires.ftc.teamcode.commandbase.CommandOpMode;
import org.firstinspires.ftc.teamcode.commandbase.Robot;

@Config
@Configurable
//one single Teleop Class that handles all Teleop Commands, then added alliance through constructor
public class Teleop extends CommandOpMode {

    MultipleTelemetry multipleTelemetry;

    Robot robot;
    final Alliance alliance;

    public boolean hold = false;
    public double speed = 1;

    public Teleop(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void init() { //what happens at initialization
        robot = new Robot(hardwareMap, alliance);
        robot.follower.setStartingPose(defaultPose);

        multipleTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
    }

    @Override
    public void init_loop() { //what happens when looping on init
        if (gamepad1.xWasPressed()) {
            robot.slides.resetSlides();
        }
    }

    @Override
    public void start() { //what happens when start is pressed
        robot.periodic();
        robot.limelight.start();
        robot.follower.startTeleOpDrive(true);
    }

    @Override
    public void loop() { //what happens during the whole match - what runs in the background
        robot.periodic();

        //if the robot is not holding position, run the Teleop Drive
        //if the alliance is Blue, then set the offset heading to PI rad. If not, then keep it 0 (for red)
        if (!hold) {
            robot.follower.setTeleOpDrive(speed * -gamepad1.left_stick_y, speed * -gamepad1.left_stick_x, speed * -gamepad1.right_stick_x, false, robot.alliance == Alliance.BLUE ? Math.toRadians(180) : 0);
        }

        if (gamepad1.startWasPressed())
            robot.resetHeading();

        if (gamepad1.xWasPressed())
            robot.slides.resetSlides();

        if (gamepad1.left_bumper)
            speed = 0.5;
        else
            speed = 1.0;

        if (gamepad1.yWasPressed()) {
            hold = !hold;

            if (hold) {
                robot.follower.holdPoint(new BezierPoint(robot.follower.getPose()), robot.follower.getHeading(), false);
            } else {
                robot.follower.startTeleopDrive();
            }
        }
        updateTelemetry();

    }

    @Override
    public void stop() { //what happens when stop button is clicked
        robot.saveEnd();
    }

    public void updateTelemetry() {
        multipleTelemetry.addData("LoopTime Hz", robot.getLoopTimeHz());
        multipleTelemetry.addLine();
        multipleTelemetry.addData("Follower Pose", robot.follower.getPose().toString());
        multipleTelemetry.addLine();
        multipleTelemetry.addData("Right Slide Pos", robot.slides.getRightPosition());
        multipleTelemetry.addData("Left Slide Pos", robot.slides.getLeftPosition());
        multipleTelemetry.addData("Slides Target", robot.slides.getTarget());
        multipleTelemetry.addData("Latch Closed", robot.latch.isDown());
        multipleTelemetry.addData("Hold Position", hold);
        multipleTelemetry.update();
    }
}
