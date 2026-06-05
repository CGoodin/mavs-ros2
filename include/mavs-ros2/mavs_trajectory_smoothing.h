#ifndef MAVS_TRAJECTORY_SMOOTHING_H
#define MAVS_TRAJECTORY_SMOOTHING_H
#include <vector>
#include <cmath>
#include <stdexcept>
#include <glm/glm.hpp>

// ---------------------------------------------------------------------------
// Trajectory smoother & one-step predictor
// ---------------------------------------------------------------------------
// Uses a simple weighted moving-average (Gaussian kernel) for positions and a
// circular weighted mean for headings so that angle wrap-around is handled
// correctly (e.g. smoothing across the 0/2pi boundary).
//
// Prediction extrapolates with a least-squares linear fit over the last
// 'fitWindow' smoothed samples, so short-term acceleration is captured.
// ---------------------------------------------------------------------------

namespace trajectory {

// ---- helpers ---------------------------------------------------------------

// Wrap angle to [-pi, pi]
inline float WrapAngle(float a) {
    while (a > M_PI) a -= 2.f * M_PI;
    while (a < -M_PI) a += 2.f * M_PI;
    return a;
}

// Gaussian kernel weight for index offset 'd' given half-width sigma
inline float GaussW(int d, float sigma) {
    return std::exp(-0.5f * (d * d) / (sigma * sigma));
}

// ---- smoothing -------------------------------------------------------------

/**
    * Smooth a position series with a Gaussian moving-average window.
    *
    * @param positions  Raw positions sampled at uniform time intervals.
    * @param sigma      Kernel half-width in *samples*. Larger = more smoothing.
    *                   A value of 1.5–3.0 works well for light noise.
    * @return           Smoothed position series (same length as input).
    */
std::vector<glm::vec2> SmoothPositions(const std::vector<glm::vec2>& positions, float sigma = 2.0f){
    const int n = static_cast<int>(positions.size());
    const int hw = static_cast<int>(std::ceil(3.f * sigma)); // 3-sigma cutoff
    std::vector<glm::vec2> out(n);

    for (int i = 0; i < n; ++i) {
        glm::vec2 acc(0.f);
        float     wsum = 0.f;
        for (int d = -hw; d <= hw; ++d) {
            int j = i + d;
            if (j < 0 || j >= n) continue;
            float w = GaussW(d, sigma);
            acc += w * positions[j];
            wsum += w;
        }
        out[i] = acc / wsum;
    }
    return out;
}

/**
    * Smooth a heading series using a circular (angular) weighted mean so that
    * wrap-around artefacts (e.g. averaging 350 degrees and 10 degrees correctly to 0 degrees) are
    * avoided.
    *
    * @param headings   Raw headings in radians, sampled at uniform time intervals.
    * @param sigma      Gaussian kernel half-width in samples.
    * @return           Smoothed heading series (same length as input).
    */
std::vector<float> SmoothHeadings(const std::vector<float>& headings, float sigma = 2.0f){

    const int n = static_cast<int>(headings.size());
    const int hw = static_cast<int>(std::ceil(3.f * sigma));
    std::vector<float> out(n);

    for (int i = 0; i < n; ++i) {
        float sinSum = 0.f, cosSum = 0.f;
        for (int d = -hw; d <= hw; ++d) {
            int j = i + d;
            if (j < 0 || j >= n) continue;
            float w = GaussW(d, sigma);
            sinSum += w * std::sin(headings[j]);
            cosSum += w * std::cos(headings[j]);
        }
        out[i] = std::atan2(sinSum, cosSum); // result in [-pi, pi]
    }
    return out;
}

// ---- prediction ------------------------------------------------------------

struct Prediction {
    glm::vec2 position; ///< Projected position at t + 1 step
    float     heading;  ///< Projected heading at t + 1 step (radians)
};

/**
    * Fit a least-squares linear trend over the last 'fitWindow' points of a
    * smoothed series and project one step ahead.
    *
    * For positions:  fits x(t) and y(t) independently.
    * For headings:   fits the *unwrapped* angle series to avoid discontinuities.
    *
    * @param smoothPos     Smoothed position series.
    * @param smoothHead    Smoothed heading series (radians).
    * @param fitWindow     Number of trailing samples used for the linear fit.
    *                      5–10 is a good starting point.
    * @return              Predicted position and heading one time-step ahead.
    */
Prediction PredictNext(const std::vector<glm::vec2>& smoothPos, const std::vector<float>& smoothHead, int fitWindow = 7){

    if (smoothPos.size() != smoothHead.size())
        throw std::invalid_argument("positions and headings must have equal length");

    const int n = static_cast<int>(smoothPos.size());
    if (n < 2)
        throw std::invalid_argument("need at least 2 samples to predict");

    fitWindow = std::min(fitWindow, n);
    const int start = n - fitWindow; // first index of the fit window

    // ---- build time indices centered at 0 for numerical stability ----------
    // t values: 0, 1, 2, ..., fitWindow-1  -> predict at fitWindow
    float sumT = 0.f, sumT2 = 0.f;
    glm::vec2 sumP(0.f);
    //float sumHUnwrapped = 0.f;

    // Unwrap headings within the fit window
    std::vector<float> unwrapped(fitWindow);
    unwrapped[0] = smoothHead[start];
    for (int k = 1; k < fitWindow; ++k) {
        float delta = WrapAngle(smoothHead[start + k] - unwrapped[k - 1]);
        unwrapped[k] = unwrapped[k - 1] + delta;
    }

    float sumHU = 0.f;
    glm::vec2 sumTP(0.f);
    float sumTHU = 0.f;

    for (int k = 0; k < fitWindow; ++k) {
        float t = static_cast<float>(k);
        sumT += t;
        sumT2 += t * t;
        sumP += smoothPos[start + k];
        sumTP += t * smoothPos[start + k];
        sumHU += unwrapped[k];
        sumTHU += t * unwrapped[k];
    }

    float fw = static_cast<float>(fitWindow);
    float den = fw * sumT2 - sumT * sumT; // always > 0 for fitWindow >= 2

    // Slope and intercept for positions
    glm::vec2 slope = (fw * sumTP - sumT * sumP) / den;
    glm::vec2 intercept = (sumP - slope * sumT) / fw;

    // Slope and intercept for heading
    float hSlope = (fw * sumTHU - sumT * sumHU) / den;
    float hIntercept = (sumHU - hSlope * sumT) / fw;

    // Predict at t = fitWindow (one step beyond the last sample)
    float tPred = static_cast<float>(fitWindow);

    Prediction pred;
    pred.position = intercept + slope * tPred;
    pred.heading = WrapAngle(hIntercept + hSlope * tPred);
    return pred;
}

// ---- convenience wrapper ---------------------------------------------------

/**
    * One-call entry point: smooth both series then predict the next step.
    *
    * @param positions   Raw position samples (uniform time steps).
    * @param headings    Raw heading samples in radians (same length).
    * @param sigma       Gaussian smoothing kernel width in samples.
    * @param fitWindow   Trailing-window size for the linear extrapolation.
 */
Prediction SmoothAndPredict(const std::vector<glm::vec2>& positions, const std::vector<float>& headings, float sigma = 2.0f, int   fitWindow = 7){
    auto sp = SmoothPositions(positions, sigma);
    auto sh = SmoothHeadings(headings, sigma);
    return PredictNext(sp, sh, fitWindow);
}

} // namespace Trajectory

#endif