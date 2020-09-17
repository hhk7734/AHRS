/*
 * MIT License
 *
 * Copyright (c) 2020 Hyeonki Hong <hhk7734@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "ahrs.h"

#include <math.h>

namespace lot {

enum { EKF_X = 0, EKF_Y, EKF_Z, EKF_W };

static const float EKF_Q_ROT = 0.1f;
static const float EKF_R_ACC = 200.0f;
static const float EKF_R_MAG = 2000.0f;

void AHRS::get_cross_product(float *a, float *b, float *a_cross_b) {
    a_cross_b[EKF_X] = a[EKF_Y] * b[EKF_Z] - a[EKF_Z] * b[EKF_Y];
    a_cross_b[EKF_Y] = a[EKF_Z] * b[EKF_X] - a[EKF_X] * b[EKF_Z];
    a_cross_b[EKF_Z] = a[EKF_X] * b[EKF_Y] - a[EKF_Y] * b[EKF_X];
}

void AHRS::get_unit_vector(float *vector, uint16_t vector_size) {
    double sq_sum = 0;
    for(int i = 0; i < vector_size; ++i) { sq_sum += (vector[i] * vector[i]); }
    float norm = sqrt(sq_sum);

    for(int i = 0; i < vector_size; ++i) { vector[i] /= norm; }
}

void AHRS::predict(float *half_delta_angle) {
    /*
     * x_(k|k-1) = f(x_(k-1|k-1))
     */
    float delta_quaternion[4];
    delta_quaternion[EKF_X] = half_delta_angle[EKF_Z] * quaternion[EKF_Y]
                              - half_delta_angle[EKF_Y] * quaternion[EKF_Z]
                              + half_delta_angle[EKF_X] * quaternion[EKF_W];

    delta_quaternion[EKF_Y] = -half_delta_angle[EKF_Z] * quaternion[EKF_X]
                              + half_delta_angle[EKF_X] * quaternion[EKF_Z]
                              + half_delta_angle[EKF_Y] * quaternion[EKF_W];

    delta_quaternion[EKF_Z] = half_delta_angle[EKF_Y] * quaternion[EKF_X]
                              - half_delta_angle[EKF_X] * quaternion[EKF_Y]
                              + half_delta_angle[EKF_Z] * quaternion[EKF_W];

    delta_quaternion[EKF_W] = -half_delta_angle[EKF_X] * quaternion[EKF_X]
                              - half_delta_angle[EKF_Y] * quaternion[EKF_Y]
                              - half_delta_angle[EKF_Z] * quaternion[EKF_Z];

    quaternion[EKF_X] += delta_quaternion[EKF_X];
    quaternion[EKF_Y] += delta_quaternion[EKF_Y];
    quaternion[EKF_Z] += delta_quaternion[EKF_Z];
    quaternion[EKF_W] += delta_quaternion[EKF_W];

    /*
     * F = df/dx
     */
    F[0][0] = 1.0;
    F[0][1] = half_delta_angle[EKF_Z];
    F[0][2] = -half_delta_angle[EKF_Y];
    F[0][3] = half_delta_angle[EKF_X];

    F[1][0] = -half_delta_angle[EKF_Z];
    F[1][1] = 1.0;
    F[1][2] = half_delta_angle[EKF_X];
    F[1][3] = half_delta_angle[EKF_Y];

    F[2][0] = half_delta_angle[EKF_Y];
    F[2][1] = -half_delta_angle[EKF_X];
    F[2][2] = 1.0;
    F[2][3] = half_delta_angle[EKF_Z];

    F[3][0] = -half_delta_angle[EKF_X];
    F[3][1] = -half_delta_angle[EKF_Y];
    F[3][2] = -half_delta_angle[EKF_Z];
    F[3][3] = 1.0;

    /*
     * P_(k|k-1) = F*P_(k-1|k-1)*F^T + Q
     * K is used instead of F*P_(k-1|k-1)
     */
    for(int i = 0; i < 4; ++i) {
        for(int j = 0; j < 4; ++j) {
            K[i][j] = F[i][0] * P[0][j] + F[i][1] * P[1][j] + F[i][2] * P[2][j]
                      + F[i][3] * P[3][j];
        }
    }

    for(int i = 0; i < 4; ++i) {
        for(int j = 0; j < 4; ++j) {
            P[i][j] = K[i][0] * F[j][0] + K[i][1] * F[j][1] + K[i][2] * F[j][2]
                      + K[i][3] * F[j][3];
        }
        P[i][i] += EKF_Q_ROT;
    }

    get_unit_vector(quaternion, 4);
}

void AHRS::update(float *unit_acc, float *unit_acc_cross_mag) {
    /*
     * y = z - h(x_(k|k-1))
     */
    y[0] = unit_acc[EKF_X]
           - 2
                 * (quaternion[EKF_X] * quaternion[EKF_Z]
                    - quaternion[EKF_Y] * quaternion[EKF_W]);
    y[1] = unit_acc[EKF_Y]
           - 2
                 * (quaternion[EKF_X] * quaternion[EKF_W]
                    + quaternion[EKF_Y] * quaternion[EKF_Z]);
    y[2] = unit_acc[EKF_Z]
           - (1
              - 2
                    * (quaternion[EKF_X] * quaternion[EKF_X]
                       + quaternion[EKF_Y] * quaternion[EKF_Y]));
    y[3] = unit_acc_cross_mag[EKF_X]
           - 2
                 * (quaternion[EKF_X] * quaternion[EKF_Y]
                    + quaternion[EKF_Z] * quaternion[EKF_W]);
    y[4] = unit_acc_cross_mag[EKF_Y]
           - (1
              - 2
                    * (quaternion[EKF_X] * quaternion[EKF_X]
                       + quaternion[EKF_Z] * quaternion[EKF_Z]));
    y[5] = unit_acc_cross_mag[EKF_Z]
           - 2
                 * (quaternion[EKF_Y] * quaternion[EKF_Z]
                    - quaternion[EKF_X] * quaternion[EKF_W]);

    /*
     * H = dh/dx
     */
    H[0][0] = 2 * quaternion[EKF_Z];
    H[0][1] = -2 * quaternion[EKF_W];
    H[0][2] = 2 * quaternion[EKF_X];
    H[0][3] = -2 * quaternion[EKF_Y];

    H[1][0] = -H[0][1];
    H[1][1] = H[0][0];
    H[1][2] = -H[0][3];
    H[1][3] = H[0][2];

    H[2][0] = -H[0][2];
    H[2][1] = H[0][3];
    H[2][2] = H[0][0];
    H[2][3] = H[1][0];

    H[3][0] = H[1][2];
    H[3][1] = H[0][2];
    H[3][2] = H[1][0];
    H[3][3] = H[0][0];

    H[4][0] = H[2][0];
    H[4][1] = H[1][2];
    H[4][2] = -H[0][0];
    H[4][3] = H[1][0];

    H[5][0] = H[0][1];
    H[5][1] = H[0][0];
    H[5][2] = H[1][2];
    H[5][3] = H[2][0];

    /*
     * S = H*P_(k|k-1)*(H^T) + R
     * K is used instead of P_(k|k-1)*(H^T)
     */
    for(int i = 0; i < 4; ++i) {
        for(int j = 0; j < 6; ++j) {
            K[i][j] = P[i][0] * H[j][0] + P[i][1] * H[j][1] + P[i][2] * H[j][2]
                      + P[i][3] * H[j][3];
        }
    }
    for(int i = 0; i < 6; ++i) {
        for(int j = 0; j < 6; ++j) {
            S[i][j] = H[i][0] * K[0][j] + H[i][1] * K[1][j] + H[i][2] * K[2][j]
                      + H[i][3] * K[3][j];
        }
        if(i < 3) S[i][i] += EKF_R_ACC;
        else
            S[i][i] += EKF_R_MAG;
    }

    /*
     * K = P_(k|k-1)*(H^T)*(S^-1)
     * K*S = P_(k|k-1)*(H^T)
     *
     * Cholesky decomposition S=L*(L^*T)  (L^*T == conjugate transpose L)
     * S is used instead of L
     */
    for(int i = 0; i < 6; ++i) {
        for(int j = 0; j < i; ++j) {
            for(int k = 0; k < j; ++k) { S[i][j] -= S[i][k] * S[j][k]; }
            S[i][j] /= S[j][j];

            S[i][i] -= S[i][j] * S[i][j];
        }
        S[i][i] = sqrt(S[i][i]);
    }

    /*
     * L*(L^*T)*(K^T) = (P_(k|k-1)*(H^T))^T
     * L*A = (P_(k|k-1)*(H^T))^T
     * K is used instead of A
     */
    for(int k = 0; k < 4; ++k) {
        for(int i = 0; i < 6; ++i) {
            for(int j = 0; j < i; ++j) { K[k][i] -= S[i][j] * K[k][j]; }
            K[k][i] /= S[i][i];
        }
    }

    /*
     * (L^*T)*(K^T) = A
     */
    for(int k = 0; k < 4; ++k) {
        for(int i = 0; i < 6; ++i) {
            for(int j = 0; j < i; ++j) {
                K[k][5 - i] -= S[5 - j][5 - i] * K[k][5 - j];
            }
            K[k][5 - i] /= S[5 - i][5 - i];
        }
    }

    /*
     * x = x + K*y
     */
    for(int i = 0; i < 4; ++i) {
        for(int j = 0; j < 6; ++j) { quaternion[i] += (K[i][j] * y[j]); }
    }

    get_unit_vector(quaternion, 4);

    /*
     * P_(k|k) = (I-K*H)*P_(k|k-1)
     * F is used instead of K*H
     */
    for(int i = 0; i < 4; ++i) {
        for(int j = 0; j < 4; ++j) {
            F[i][j] = K[i][0] * H[0][j] + K[i][1] * H[1][j] + K[i][2] * H[2][j]
                      + K[i][3] * H[3][j];
        }
    }

    for(int i = 0; i < 4; ++i) {
        for(int j = 0; j < 4; ++j) {
            K[i][j] = F[i][0] * P[0][j] + F[i][1] * P[1][j] + F[i][2] * P[2][j]
                      + F[i][3] * P[3][j];
        }
    }

    for(int i = 0; i < 4; ++i) {
        for(int j = 0; j < 4; ++j) { P[i][j] -= K[i][j]; }
    }
}

void AHRS::get_quaternion(float *quaternion) {
    quaternion[EKF_X] = this->quaternion[EKF_X];
    quaternion[EKF_Y] = this->quaternion[EKF_Y];
    quaternion[EKF_Z] = this->quaternion[EKF_Z];
    quaternion[EKF_W] = this->quaternion[EKF_W];
}

void AHRS::get_RPY(float *rpy) {
    /*
     * Tait-Bryan angles Z-Y-X rotation
     * roll  = atan2( 2*(qy*qz+qx*qw) , 1-2*(qx^2+qy^2) )
     * pitch = asin ( 2*(qy*qw-qx*qz) )
     * yaw   = atan2( 2*(qx*qy+qz*qw) , 1-2*(qy^2+qz^2) )
     */
    rpy[EKF_X] = atan2(2
                           * (quaternion[EKF_Y] * quaternion[EKF_Z]
                              + quaternion[EKF_X] * quaternion[EKF_W]),
                       1
                           - 2
                                 * (quaternion[EKF_X] * quaternion[EKF_X]
                                    + quaternion[EKF_Y] * quaternion[EKF_Y]));
    rpy[EKF_Y] = -asin(2
                       * (quaternion[EKF_Y] * quaternion[EKF_W]
                          - quaternion[EKF_X] * quaternion[EKF_Z]));
    rpy[EKF_Z] = -atan2(2
                            * (quaternion[EKF_X] * quaternion[EKF_Y]
                               + quaternion[EKF_Z] * quaternion[EKF_W]),
                        1
                            - 2
                                  * (quaternion[EKF_Y] * quaternion[EKF_Y]
                                     + quaternion[EKF_Z] * quaternion[EKF_Z]));
}

void AHRS::get_frame_rotation(float *vector) {
    float temp[3];
    temp[EKF_X] = (1
                   - 2
                         * (quaternion[EKF_Y] * quaternion[EKF_Y]
                            + quaternion[EKF_Z] * quaternion[EKF_Z]))
                      * vector[EKF_X]
                  + (2
                     * (quaternion[EKF_X] * quaternion[EKF_Y]
                        + quaternion[EKF_Z] * quaternion[EKF_W]))
                        * vector[EKF_Y]
                  + (2
                     * (quaternion[EKF_X] * quaternion[EKF_Z]
                        - quaternion[EKF_Y] * quaternion[EKF_W]))
                        * vector[EKF_Z];
    temp[EKF_Y] = (2
                   * (quaternion[EKF_X] * quaternion[EKF_Y]
                      - quaternion[EKF_Z] * quaternion[EKF_W]))
                      * vector[EKF_X]
                  + (1
                     - 2
                           * (quaternion[EKF_X] * quaternion[EKF_X]
                              + quaternion[EKF_Z] * quaternion[EKF_Z]))
                        * vector[EKF_Y]
                  + (2
                     * (quaternion[EKF_Y] * quaternion[EKF_Z]
                        + quaternion[EKF_X] * quaternion[EKF_W]))
                        * vector[EKF_Z];
    temp[EKF_Z] = (2
                   * (quaternion[EKF_X] * quaternion[EKF_Z]
                      + quaternion[EKF_Y] * quaternion[EKF_W]))
                      * vector[EKF_X]
                  + (2
                     * (quaternion[EKF_Y] * quaternion[EKF_Z]
                        - quaternion[EKF_X] * quaternion[EKF_W]))
                        * vector[EKF_Y]
                  + (1
                     - 2
                           * (quaternion[EKF_X] * quaternion[EKF_X]
                              + quaternion[EKF_Y] * quaternion[EKF_Y]))
                        * vector[EKF_Z];

    vector[EKF_X] = temp[EKF_X];
    vector[EKF_Y] = temp[EKF_Y];
    vector[EKF_Z] = temp[EKF_Z];
}
}    // namespace lot
