package org.firstinspires.ftc.teamcode.commandbase;

import static com.pedropathing.ivy.groups.Groups.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Latch;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.SlideArm;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Slides;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;
import java.util.concurrent.TimeUnit;

//class where all robot mechanisms and routines are handled
// Structure: Subsystems & Commands -> Robot Class -> Teleop
public class Robot {
    public final Intake intake;
    public final SlideArm slideArm;
    public final Latch latch;
    public final Slides slides;
    public final Follower follower;
    public Alliance alliance;

    private final List<LynxModule> hubs;
    private final com.pedropathing.utils.Timer loopTimer = new com.pedropathing.utils.Timer();
    public static Pose endPose = new Pose(0, 0, 0);
    public double loops = 0, lastLoop = 0, loopTime = 0;

    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;
        intake = new Intake(hardwareMap);
        latch = new Latch(hardwareMap);
        slides = new Slides(hardwareMap);
        slideArm = new SlideArm(hardwareMap);
        follower = Constants.create(hardwareMap);

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        loopTimer.reset();

        periodic();
    }

    public void periodic() {
        loops++;

        if (loops > 10) { //for ref, 50hz is a solid loop time
            double now = loopTimer.get(TimeUnit.MILLISECONDS);
            loopTime = (now - lastLoop) / loops;
            lastLoop = now;
            loops = 0;
        }

        follower.update();
        slides.periodic();
    }

    public void saveEnd() { //use at end of all autos
        endPose = follower.pose();
    }

    public void resetHeading() {
        follower.setPose(follower.pose().withHeading(alliance == Alliance.BLUE ? Math.toRadians(180) : 0));
    }

    public CommandBuilder intake() {
        return sequential(
                latch.toOpenPos(),
                intake.on()
        );
    }

    public String getMechanismCurrent() {
        return intake.getCurrent() + "/n" + slides.getRightCurrent() + "/n" + slides.getLeftCurrent();
    }

    public double getLoopTimeMs() {
        return loopTime;
    }

    public double getLoopTimeHz() {
        return 1000 / loopTime;
    }
}
