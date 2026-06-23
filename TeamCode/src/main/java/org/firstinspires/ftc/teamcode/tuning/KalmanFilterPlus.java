package org.firstinspires.ftc.teamcode.tuning;

import com.pedropathing.control.KalmanFilterParameters;
import com.pedropathing.control.NoiseFilter;


public class KalmanFilterPlus implements NoiseFilter {
    private KalmanFilterParameters parameters;
    private double state;
    private double variance;
    private double kalmanGain;
    private double previousState;
    private double previousVariance;


    public KalmanFilterPlus(KalmanFilterParameters parameters) {
        this.parameters = parameters;
        reset();
    }

    public KalmanFilterPlus(KalmanFilterParameters parameters, double startState, double startVariance, double startGain) {
        this.parameters = parameters;
        reset(startState, startVariance, startGain);
    }

    public void reset(double startState, double startVariance, double startGain) {
        state = startState;
        previousState = startState;
        variance = startVariance;
        previousVariance = startVariance;
        kalmanGain = startGain;
    }

    public void reset() {
        reset(0, 1, 1);
    }

    public void update(double updateData, double updateProjection) {
        state = previousState + updateData;
        variance = previousVariance + parameters.modelCovariance;
        kalmanGain = variance / (variance + parameters.dataCovariance);
        state += kalmanGain * (updateProjection - state);
        variance *= (1.0 - kalmanGain);
        previousState = state;
        previousVariance = variance;
    }

    public void update(double updateData) {
        state = previousState + updateData;
        variance = previousVariance + parameters.modelCovariance;
        previousState = state;
        previousVariance = variance;
    }

    public double getState() {
        return state;
    }

    public double getVariance() { return variance; }

    /**
     * This method outputs the current state, variance, and Kalman gain of the filter as a string array.
     * @return A string array containing the current state, variance, and Kalman gain.
     */
    public String[] output() {
        return new String[]{
                "State: " + state,
                "Variance: " + variance,
                "Kalman Gain: " + kalmanGain
        };
    }
}
