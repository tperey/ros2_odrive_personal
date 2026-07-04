#ifndef ACCEL_KF
#define ACCEL_KF

class AccelKF {
public:
    AccelKF(float initialPos, float initialVel, float initialAcc,
            float measNoise, float processNoise[3][3], float passed_dt,
            float initP[3][3]);

    // Update filter with encoder measurement (position only)
    void update(float z);

    // Getters
    float getPosition() const;
    float getVelocity() const;
    float getAcceleration() const;

private:
    float x[3];       // State: [position, velocity, acceleration]
    float P[3][3];    // Covariance
    float Q[3][3];    // Process noise
    float R;          // Measurement noise (position)
    float dt;         // Constant dt
    float F[3][3];    // State transition matrix

    // Predict next state based on constant acceleration model
    void _predict();

    // Helper: matrix multiplication F*P*F^T + Q
    void _predictCovariance();
};

#endif
