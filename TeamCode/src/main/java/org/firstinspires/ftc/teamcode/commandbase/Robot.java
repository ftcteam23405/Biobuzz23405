package org.firstinspires.ftc.teamcode.commandbase;

import static com.pedropathing.ivy.groups.Groups.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Latch;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Slides;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

//class where all robot mechanisms and routines are handled
// Structure: Subsystems & Commands -> Robot Class -> Teleop
public class Robot {
    public final Intake intake;
    public final Limelight limelight;
    public final Latch latch;
    public final Slides slides;
    public final Follower follower;
    public Alliance alliance;

    private final List<LynxModule> hubs;
    private final com.pedropathing.util.Timer loop = new Timer();
    public static Pose defaultPose = new Pose(8 + 24, 6.25 + 24, 0);
    public double loops = 0, lastLoop = 0, loopTime = 0;

    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;
        intake = new Intake(hardwareMap);
        limelight = new Limelight(hardwareMap, alliance);
        latch = new Latch(hardwareMap);
        slides = new Slides(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        loop.resetTimer();

        periodic();
    }

    public void periodic() {
        loops++;

        if (loops > 10) { //for ref, 50hz is a solid loop time
            double now = loop.getElapsedTime();
            loopTime = (now - lastLoop) / loops;
            lastLoop = now;
            loops = 0;
        }

        follower.update();
        slides.periodic();
    }

    public void saveEnd() {
        defaultPose = follower.getPose();
    }

    public CommandBuilder intake() {
        return sequential(
                latch.down(),
                intake.on(),
                Commands.waitMs(500)
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
