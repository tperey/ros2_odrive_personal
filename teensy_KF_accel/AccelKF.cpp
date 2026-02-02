#include "AccelKF.hpp"

// Constructor
AccelKF::AccelKF(float initialPos, float initialVel, float initialAcc,
                 float measNoise, float processNoise[3][3], float passed_dt,
                 float initP[3][3]) {
    x[0] = initialPos;
    x[1] = initialVel;
    x[2] = initialAcc;

    dt = passed_dt;
    R = measNoise;

    // State transition matrix
    F[0][0] = 1.0;
    F[0][1] = dt;
    F[0][2] = 0.5*dt*dt;
    F[1][0] = 0.0;
    F[1][1] = 1.0;
    F[1][2] = dt;
    F[2][0] = 0.0;
    F[2][1] = 0.0;
    F[2][2] = 1.0;

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            P[i][j] = initP[i][j]; // initial uncertainty
            Q[i][j] = processNoise[i][j];
        }
}

// Predict step
void AccelKF::_predict() {
    // State transition (constant acceleration)
    float x_pred[3];
    x_pred[0] = x[0] + x[1]*dt + 0.5f*x[2]*dt*dt;
    x_pred[1] = x[1] + x[2]*dt;
    x_pred[2] = x[2];

    // Predict covariance
    _predictCovariance();

    // Update state
    for (int i = 0; i < 3; i++)
        x[i] = x_pred[i];
}

// Update step (encoder measurement)
void AccelKF::update(float z) {
    /* PREDICTION STEP*/
    _predict();

    /* UPDATE STEP */
    float y = z - x[0];       // residual
    float S = P[0][0] + R;    // residual covariance

    float K[3];               // Kalman gain
    for (int i = 0; i < 3; i++)
        K[i] = P[i][0] / S;

    // Update state
    for (int i = 0; i < 3; i++)
        x[i] += K[i] * y;

    // Update covariance
    float P_new[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            P_new[i][j] = P[i][j] - K[i] * P[0][j];

    for (int i = 0; i < 3; i++)  // Copy back
        for (int j = 0; j < 3; j++)
            P[i][j] = P_new[i][j];
}

// Helper: predict covariance
void AccelKF::_predictCovariance() {
    float P_pred[3][3] = {0.0f};

    // Compute P_pred = F * P * F^T + Q
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P_pred[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                for (int l = 0; l < 3; l++) {
                    P_pred[i][j] += F[i][k] * P[k][l] * F[j][l]; // correct F*P*F^T
                }
            }
            // Add process noise
            P_pred[i][j] += Q[i][j];
        }
    }

    // Copy back to P
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            P[i][j] = P_pred[i][j];
}

// Getters
float AccelKF::getPosition() const { return x[0]; }
float AccelKF::getVelocity() const { return x[1]; }
float AccelKF::getAcceleration() const { return x[2]; }
