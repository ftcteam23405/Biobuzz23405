package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathPoint;

public class WiggleInterpolator implements HeadingInterpolator {

    private final double constantHeading;

    public double StuckSpeedThreshold = 0.5;

    public double angleBump = 15; // degrees

    public WiggleInterpolator(boolean wallOnLeft, double constantHeading){
        if (wallOnLeft){
            angleBump = -angleBump;
        }
        this.constantHeading = constantHeading;
    }

    @Override
    public double interpolate(PathPoint closestPoint) {
        if (Math.abs(follower.getVelocity().getMagnitude()) < StuckSpeedThreshold){
            return constantHeading +  Math.toRadians(angleBump);
        } else {
            return constantHeading;
        }
    }
}
