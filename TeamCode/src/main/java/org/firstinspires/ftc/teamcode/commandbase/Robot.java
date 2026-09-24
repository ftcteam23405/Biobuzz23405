package org.firstinspires.ftc.teamcode.commandbase;

import static com.pedropathing.ivy.groups.Groups.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Latch;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;
import java.util.concurrent.TimeUnit;

//class where all robot mechanisms and routines are handled
// Structure: Subsystems & Commands -> Robot Class -> Teleop
public class Robot {
    public final Intake intake;
    public final Latch latch;
    public final Shooter shooter;
    public final Limelight limelight;
    public final Follower follower;
    public Alliance alliance;

    private final List<LynxModule> hubs;
    private final com.pedropathing.utils.Timer loopTimer = new com.pedropathing.utils.Timer();
    private static Pose endPose = new Pose(0, 0, 0);
    public double loops = 0, lastLoop = 0, loopTime = 0;

    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;
        intake = new Intake(hardwareMap);
        latch = new Latch(hardwareMap);
        shooter = new Shooter(hardwareMap);
        limelight = new Limelight(hardwareMap, alliance);
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
        limelight.periodic();

        // only read tags while tracking, distanceToGoal() switches the limelight to the tags pipeline
        if (shooter.isTracking())
            shooter.setVelocityFromDistance(follower.pose(), alliance, limelight.distanceToGoal());
        shooter.periodic();
    }

    public void setEnd() { //use at end of all autos
        endPose = follower.pose();
    }
    public Pose getEnd() {
        return endPose;
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


    public double getLoopTimeMs() {
        return loopTime;
    }

    public double getLoopTimeHz() {
        return 1000 / loopTime;
    }
}
