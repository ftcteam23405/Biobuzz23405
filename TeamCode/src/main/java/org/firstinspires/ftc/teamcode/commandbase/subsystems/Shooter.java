package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.commandbase.Alliance;

import smile.interpolation.Interpolation;
import smile.interpolation.LinearInterpolation;

@Configurable
@Config
public class Shooter {


    public static double kS = 0.08;      // power to overcome friction
    public static double kV = 0.00039;   // power per tick/s
    public static double kP = 0.01;      // power per tick/s of error
    public static double tolerance = 50; // ticks/s


    public static double redGoalX = 62;
    public static double blueGoalX = 82;
    public static double audienceGoalY = 62.6;
    public static double farGoalY = 81.4;

    public static double limelightWeight = 0.7; // 0 = odometry only, 1 = limelight only
    public static double smoothing = 0.3;       // 0 = never updates, 1 = no smoothing


    // straight-line distance to the closest goal (inches) -> flywheel velocity (ticks/s)
    // placeholder values, fill these in with ShooterTuner. distances must be increasing
    private static final double[] DISTANCES = {40, 60, 80, 100, 120};
    private static final double[] VELOCITIES = {1200, 1250, 1300, 1350, 1400};
    private static final Interpolation velocityTable = new LinearInterpolation(DISTANCES, VELOCITIES);

    private final DcMotorEx topMotor, bottomMotor;

    private double targetVelocity = 0;
    private boolean tracking = false; // true = velocity follows distance to the goal every loop

    // filtered straight-line distance from the robot to the closest goal
    private double goalDistance = 0;
    private boolean filterStarted = false;

    // raw readings from the last update, for telemetry
    private double odometryDistance = 0;
    private double limelightDistance = Double.NaN;

    public Shooter(HardwareMap hardwareMap) {
        topMotor = hardwareMap.get(DcMotorEx.class, "shooterMotorTop");
        bottomMotor = hardwareMap.get(DcMotorEx.class, "shooterMotorBottom");

        topMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        topMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        stop();
    }

    // spin at a fixed velocity, stops distance tracking
    public void setTargetVelocity(double velocity) {
        tracking = false;
        targetVelocity = Math.max(0, velocity);
    }

    // velocity follows the distance to the goal until stop() or setTargetVelocity()
    public void startTracking() {
        tracking = true;
        resetFilter(); // start fresh instead of from wherever the robot was last time
    }

    public void stop() {
        setTargetVelocity(0);
        setPower(0);
    }

    public boolean isTracking() {
        return tracking;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getVelocity() {
        return topMotor.getVelocity();
    }

    public boolean atTarget() {
        return (targetVelocity > 0) && Math.abs(targetVelocity - getVelocity()) < tolerance;
    }

    public void periodic() {
        if (targetVelocity == 0) {
            setPower(0);
            return;
        }

        double feedforward = kS + kV * targetVelocity;
        double feedback = kP * (targetVelocity - getVelocity());

        // don't reverse the flywheel to slow it down, just let it coast
        setPower(Range.clip(feedforward + feedback, 0, 1));
    }

    private void setPower(double power) {
        topMotor.setPower(power);
        bottomMotor.setPower(power);
    }


    // sets the target velocity from how far we are from the goal, using odometry + limelight
    // robot: follower pose
    // limelightDistance: from Limelight.distanceToGoal(), NaN if no tag is visible
    public void setVelocityFromDistance(Pose robot, Alliance alliance, double limelightDistance) {
        odometryDistance = robot.distance(closestGoal(robot, alliance));
        this.limelightDistance = limelightDistance;

        boolean tagDetected = !Double.isNaN(limelightDistance);

        double distance;
        if (tagDetected) {
            // the limelight doesn't drift, so it gets limelightWeight when we can see a tag
            distance = blend(odometryDistance, limelightDistance, limelightWeight);
        } else {
            // no tag, pinpoint only
            distance = odometryDistance;
        }

        setVelocityFromDistance(distance);
    }

    // sets the target velocity from a single distance reading (inches), still filtered
    public void setVelocityFromDistance(double distance) {
        if (Double.isNaN(distance)) return; // keep the last target instead of spinning at garbage

        updateFilter(distance);
        targetVelocity = velocityFor(goalDistance);
    }

    // looks up the velocity for a distance from the goal, clamped to the ends of the table
    public static double velocityFor(double distance) {
        double d = Range.clip(distance, DISTANCES[0], DISTANCES[DISTANCES.length - 1]);
        return velocityTable.interpolate(d);
    }

    // the one of our 2 cells that's closer to the robot
    public static Pose closestGoal(Pose robot, Alliance alliance) {
        double goalX = alliance == Alliance.RED ? redGoalX : blueGoalX;
        Pose audienceGoal = new Pose(goalX, audienceGoalY);
        Pose farGoal = new Pose(goalX, farGoalY);

        return robot.distance(audienceGoal) < robot.distance(farGoal) ? audienceGoal : farGoal;
    }

    // low pass filter, so one bad frame or a tag popping in and out doesn't jerk the flywheel
    private void updateFilter(double distance) {
        if (!filterStarted) {
            goalDistance = distance;
            filterStarted = true;
            return;
        }

        goalDistance = blend(goalDistance, distance, smoothing);
    }

    public void resetFilter() {
        filterStarted = false;
    }

    // weight = 0 gives a, weight = 1 gives b
    private static double blend(double a, double b, double weight) {
        return a + weight * (b - a);
    }

    public double getGoalDistance() {
        return goalDistance;
    }

    public double getOdometryDistance() {
        return odometryDistance;
    }

    // NaN if no tag was seen on the last update
    public double getLimelightDistance() {
        return limelightDistance;
    }

    // spins up and keeps adjusting velocity to the distance from the goal
    public CommandBuilder spinUp() {
        return Commands.instant(this::startTracking);
    }

    // spins up to a fixed velocity
    public CommandBuilder spinUp(double velocity) {
        return Commands.instant(() -> setTargetVelocity(velocity));
    }

    public CommandBuilder spinDown() {
        return Commands.instant(this::stop);
    }

    public CommandBuilder waitUntilAtTarget() {
        return Commands.waitUntil(this::atTarget);
    }
}
