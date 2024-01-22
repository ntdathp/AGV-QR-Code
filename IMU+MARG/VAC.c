#include <stdio.h>
#include <stdbool.h>
#include <math.h>

double qvac_[4];
double ax[5];
double ay[5];
double az[5];
double hx[5];
double hy[5];
double hz[5];
double wx[5];
double wy[5];
double wz[5];
 double wx_;
double wy_;
double wz_;
double gain_acc_static = 0.01;
double gain_mag = 0.01;
double th1a = 0.1;
double th2a = 0.99;
double biasalpha = 0.01;
double qvac[4];
int now, lastupdate;

struct Euler {
    float phi;
    float theta;
    float psi;
};

void calculateEuler(double q[4], struct Euler *euler) {
    float R[3][3];

    R[0][0] = 2 * q[0] * q[0] - 1 + 2 * q[1] * q[1];
    R[1][0] = 2 * (q[1] * q[2] - q[0] * q[3]);
    R[2][0] = 2 * (q[1] * q[3] + q[0] * q[2]);
    R[2][1] = 2 * (q[2] * q[3] - q[0] * q[1]);
    R[2][2] = 2 * q[0] * q[0] - 1 + 2 * q[3] * q[3];

    euler->phi = atan2(R[2][1], R[2][2]);
    euler->theta = -atan(R[2][0] / sqrt(1 - R[2][0] * R[2][0]));
    euler->psi = atan2(R[1][0], R[0][0]);
}




static int isInitialnized_not_empty, isInitialized_VACimpl;

void isInitialnized_not_empty_init(void)
{
  isInitialnized_not_empty = 0;
}

void VACimpl_initialize(void)
{
  isInitialnized_not_empty_init();
  isInitialized_VACimpl = 1;
}
/* Function Definitions */
/*
 * Arguments    : const double qvac_[4]
 *                double ax
 *                double ay
 *                double az
 *                double hx
 *                double hy
 *                double hz
 *                double wx
 *                double wy
 *                double wz
 *                double wx_
 *                double wy_
 *                double wz_
 *                double gain_acc_static
 *                double gain_mag
 *                double th1a
 *                double th2a
 *                double biasalpha
 *                double qvac[4]
 * Return Type  : void
 */
void VACimpl(const double qvac_[4], double ax, double ay, double az, double hx,
             double hy, double hz, double wx, double wy, double wz, double wx_,
             double wy_, double wz_, double gain_acc_static, double gain_mag,
             double th1a, double th2a, double biasalpha, double qvac[4])
{
  double rot_matr[9];
  double gp[3];
  double b_rot_matr_tmp;
  double bx;
  double by;
  double bz;
  double c_rot_matr_tmp;
  double d_rot_matr_tmp;
  double dqacc_idx_0;
  double dqacc_idx_1;
  double dqacc_idx_2;
  double dqacc_idx_3;
  double n;
  double qa_idx_0;
  double qa_idx_1;
  double qa_idx_2;
  double qw_idx_0;
  double qw_idx_1;
  double qw_idx_2;
  double qw_idx_3;
  double rot_matr_tmp;
  double t;
  int i;
  int steady;
  if (!isInitialized_VACimpl) {
    VACimpl_initialize();
  }
  bx = 0.0;
  by = 0.0;
  bz = 0.0;
  if (!isInitialnized_not_empty) {
    /* first step: initialization */
    /* ASSUMPTION: THE SYSTEM IS IN A STEADY CONFIGURATION, the gyro */
    /* output is ignored and the attitude is only determined by signal */
    /* from accelerometer and magnetometer */
    isInitialnized_not_empty = 1;
    bx = 3.3121686421112381E-170;
    bz = fabs(ax);
    if (bz > 3.3121686421112381E-170) {
      n = 1.0;
      bx = bz;
    } else {
      t = bz / 3.3121686421112381E-170;
      n = t * t;
    }
    bz = fabs(ay);
    if (bz > bx) {
      t = bx / bz;
      n = n * t * t + 1.0;
      bx = bz;
    } else {
      t = bz / bx;
      n += t * t;
    }
    bz = fabs(az);
    if (bz > bx) {
      t = bx / bz;
      n = n * t * t + 1.0;
      bx = bz;
    } else {
      t = bz / bx;
      n += t * t;
    }
    n = bx * sqrt(n);
    bz = ax / n;
    by = ay / n;
    bx = az / n;
    if (bx >= 0.0) {
      dqacc_idx_0 = sqrt((bx + 1.0) / 2.0);
      bx = sqrt(2.0 * (bx + 1.0));
      dqacc_idx_1 = -by / bx;
      dqacc_idx_2 = bz / bx;
      dqacc_idx_3 = 0.0;
    } else {
      t = sqrt(2.0 * (1.0 - bx));
      dqacc_idx_0 = -by / t;
      dqacc_idx_1 = sqrt((1.0 - bx) / 2.0);
      dqacc_idx_2 = 0.0;
      dqacc_idx_3 = bz / t;
    }
    /* this function returns the rotation matrix based on the rotation
     * quaternion */
    /* the scalar part is located at the begin of the quaternion */
    bx = dqacc_idx_0 * dqacc_idx_0;
    by = dqacc_idx_1 * dqacc_idx_1;
    bz = dqacc_idx_2 * dqacc_idx_2;
    t = dqacc_idx_3 * dqacc_idx_3;
    rot_matr[0] = ((bx + by) - bz) - t;
    rot_matr_tmp = dqacc_idx_1 * dqacc_idx_2;
    b_rot_matr_tmp = dqacc_idx_0 * dqacc_idx_3;
    rot_matr[3] = 2.0 * (rot_matr_tmp - b_rot_matr_tmp);
    c_rot_matr_tmp = dqacc_idx_1 * dqacc_idx_3;
    d_rot_matr_tmp = dqacc_idx_0 * dqacc_idx_2;
    rot_matr[6] = 2.0 * (c_rot_matr_tmp + d_rot_matr_tmp);
    rot_matr[1] = 2.0 * (rot_matr_tmp + b_rot_matr_tmp);
    bx -= by;
    rot_matr[4] = (bx + bz) - t;
    by = dqacc_idx_2 * dqacc_idx_3;
    rot_matr_tmp = dqacc_idx_0 * dqacc_idx_1;
    rot_matr[7] = 2.0 * (by - rot_matr_tmp);
    rot_matr[2] = 2.0 * (c_rot_matr_tmp - d_rot_matr_tmp);
    rot_matr[5] = 2.0 * (by + rot_matr_tmp);
    rot_matr[8] = (bx - bz) + t;
    for (i = 0; i < 3; i++) {
      gp[i] = (rot_matr[3 * i] * hx + rot_matr[3 * i + 1] * hy) +
              rot_matr[3 * i + 2] * hz;
    }
    by = gp[0] * gp[0] + gp[1] * gp[1];
    bx = sqrt(by + gp[0] * sqrt(by));
    qw_idx_0 = bx / sqrt(2.0 * by);
    qw_idx_3 = gp[1] / (1.4142135623730951 * bx);
    qvac[0] =
        ((dqacc_idx_0 * qw_idx_0 - dqacc_idx_1 * 0.0) - dqacc_idx_2 * 0.0) -
        dqacc_idx_3 * qw_idx_3;
    qvac[1] = (dqacc_idx_0 * 0.0 + qw_idx_0 * dqacc_idx_1) +
              (dqacc_idx_2 * qw_idx_3 - 0.0 * dqacc_idx_3);
    qvac[2] = (dqacc_idx_0 * 0.0 + qw_idx_0 * dqacc_idx_2) +
              (0.0 * dqacc_idx_3 - dqacc_idx_1 * qw_idx_3);
    qvac[3] = (dqacc_idx_0 * qw_idx_3 + qw_idx_0 * dqacc_idx_3) +
              (dqacc_idx_1 * 0.0 - 0.0 * dqacc_idx_2);
  } else {
    /* PREDICTION STEP */
    /* the orientation is determined from gyro output and. Here the */
    /* quaternion that describe the rate of change of orientation is */
    /* calculated */
    steady = !(fabs(sqrt((ax * ax + ay * ay) + az * az) - 9.81) > 0.1);
    if ((fabs(wx - wx_) > 0.01) || (fabs(wy - wy_) > 0.01) ||
        (fabs(wz - wz_) > 0.01)) {
      steady = false;
    }
    if ((fabs(wx) > 0.2) || (fabs(wy) > 0.2) || (fabs(wz) > 0.2)) {
      steady = false;
    }
    if (steady) {
      bx = biasalpha * wx;
      by = biasalpha * wy;
      bz = biasalpha * wz;
    }
    dqacc_idx_1 = wx - bx;
    dqacc_idx_2 = wy - by;
    dqacc_idx_3 = wz - bz;

   
    now = HAL_GetTick();
    int dt = (now - lastupdate) / 1000.0f;
    lastupdate = now;
  


    t = qvac_[0] - 0.5 * dt * (((0.0 * qvac_[0] - dqacc_idx_1 * qvac_[1]) -
                             dqacc_idx_2 * qvac_[2]) -
                            dqacc_idx_3 * qvac_[3]);
    qw_idx_0 = t;
    bx = t * t;
    t = qvac_[1] - 0.5 * dt * ((0.0 * qvac_[1] + qvac_[0] * dqacc_idx_1) +
                            (dqacc_idx_2 * qvac_[3] - qvac_[2] * dqacc_idx_3));
    qw_idx_1 = t;
    by = t * t;
    t = qvac_[2] - 0.5 * dt * ((0.0 * qvac_[2] + qvac_[0] * dqacc_idx_2) +
                            (qvac_[1] * dqacc_idx_3 - dqacc_idx_1 * qvac_[3]));
    qw_idx_2 = t;
    bz = t * t;
    t = qvac_[3] - 0.5 * dt * ((0.0 * qvac_[3] + qvac_[0] * dqacc_idx_3) +
                            (dqacc_idx_1 * qvac_[2] - qvac_[1] * dqacc_idx_2));
    bx = sqrt(((bx + by) + bz) + t * t);
    qw_idx_0 /= bx;
    qw_idx_1 /= bx;
    qw_idx_2 /= bx;
    qw_idx_3 = t / bx;
    /* CORRECTION STEP */
    /* 1) Accelerometer-Based Correction */
    /* - determine the predicted gravity and calculate the delta */
    /* quaternion */
    bx = 3.3121686421112381E-170;
    bz = fabs(ax);
    if (bz > 3.3121686421112381E-170) {
      n = 1.0;
      bx = bz;
    } else {
      t = bz / 3.3121686421112381E-170;
      n = t * t;
    }
    bz = fabs(ay);
    if (bz > bx) {
      t = bx / bz;
      n = n * t * t + 1.0;
      bx = bz;
    } else {
      t = bz / bx;
      n += t * t;
    }
    bz = fabs(az);
    if (bz > bx) {
      t = bx / bz;
      n = n * t * t + 1.0;
      bx = bz;
    } else {
      t = bz / bx;
      n += t * t;
    }
    n = bx * sqrt(n);
    /* this function returns the rotation matrix based on the rotation
     * quaternion */
    /* the scalar part is located at the begin of the quaternion */
    
    bx = qw_idx_0 * qw_idx_0;
    by = qw_idx_1 * qw_idx_1;
    bz = qw_idx_2 * qw_idx_2;
    t = qw_idx_3 * qw_idx_3;
    rot_matr[0] = ((bx + by) - bz) - t;
    rot_matr_tmp = qw_idx_1 * qw_idx_2;
    b_rot_matr_tmp = qw_idx_0 * qw_idx_3;
    rot_matr[1] = 2.0 * (rot_matr_tmp - b_rot_matr_tmp);
    c_rot_matr_tmp = qw_idx_1 * qw_idx_3;
    d_rot_matr_tmp = qw_idx_0 * qw_idx_2;
    rot_matr[2] = 2.0 * (c_rot_matr_tmp + d_rot_matr_tmp);
    rot_matr[3] = 2.0 * (rot_matr_tmp + b_rot_matr_tmp);
    bx -= by;
    rot_matr[4] = (bx + bz) - t;
    by = qw_idx_2 * qw_idx_3;
    rot_matr_tmp = qw_idx_0 * qw_idx_1;
    rot_matr[5] = 2.0 * (by - rot_matr_tmp);
    rot_matr[6] = 2.0 * (c_rot_matr_tmp - d_rot_matr_tmp);
    rot_matr[7] = 2.0 * (by + rot_matr_tmp);
    rot_matr[8] = (bx - bz) + t;
    bx = ax / n;
    by = ay / n;
    bz = az / n;
    for (i = 0; i < 3; i++) {
      gp[i] = (rot_matr[i] * bx + rot_matr[i + 3] * by) + rot_matr[i + 6] * bz;
    }
    dqacc_idx_0 = sqrt((gp[2] + 1.0) / 2.0);
    bx = sqrt(2.0 * (gp[2] + 1.0));
    dqacc_idx_1 = -gp[1] / bx;
    dqacc_idx_2 = gp[0] / bx;
    /* Adaptive Gain (accelerometer) */
    bx = fabs(n - 9.81) / 9.81;
    if (bx < th1a) {
      bx = 1.0;
    } else if ((bx > th1a) && (bx < th2a)) {
      bx = 1.0 - (bx - th1a) / (th2a - th1a);
    } else {
      bx = 0.0;
    }
    by = gain_acc_static * bx;
    if (dqacc_idx_0 < 0.9) {
      /* if it is true compute the Spherical Interpolation */
      bz = acos(dqacc_idx_0);
      bx = sin(bz);
      t = sin((1.0 - by) * bz) / bx;
      bx = sin(by * bz) / bx;
      dqacc_idx_0 = t + bx * dqacc_idx_0;
      dqacc_idx_1 = t * 0.0 + bx * dqacc_idx_1;
      dqacc_idx_2 = t * 0.0 + bx * dqacc_idx_2;
      dqacc_idx_3 = t * 0.0 + bx * 0.0;
    } else {
      bx = 3.3121686421112381E-170;
      if (dqacc_idx_0 > 3.3121686421112381E-170) {
        rot_matr_tmp = 1.0;
        bx = dqacc_idx_0;
      } else {
        rot_matr_tmp = 0;
      }
      bz = fabs(dqacc_idx_1);
      if (bz > bx) {
        t = bx / bz;
        rot_matr_tmp = rot_matr_tmp * t * t + 1.0;
        bx = bz;
      } else {
        t = bz / bx;
        rot_matr_tmp += t * t;
      }
      bz = fabs(dqacc_idx_2);
      if (bz > bx) {
        t = bx / bz;
        rot_matr_tmp = rot_matr_tmp * t * t + 1.0;
        bx = bz;
      } else {
        t = bz / bx;
        rot_matr_tmp += t * t;
      }
      rot_matr_tmp = bx * sqrt(rot_matr_tmp);
      dqacc_idx_0 = ((1.0 - by) + by * dqacc_idx_0) / rot_matr_tmp;
      bx = (1.0 - by) * 0.0;
      dqacc_idx_1 = (bx + by * dqacc_idx_1) / rot_matr_tmp;
      dqacc_idx_2 = (bx + by * dqacc_idx_2) / rot_matr_tmp;
      dqacc_idx_3 = (bx + by * 0.0) / rot_matr_tmp;
    }
    bx = sqrt(((dqacc_idx_0 * dqacc_idx_0 + dqacc_idx_1 * dqacc_idx_1) +
               dqacc_idx_2 * dqacc_idx_2) +
              dqacc_idx_3 * dqacc_idx_3);
    dqacc_idx_0 /= bx;
    dqacc_idx_1 /= bx;
    dqacc_idx_2 /= bx;
    dqacc_idx_3 /= bx;
    qa_idx_0 = ((qw_idx_0 * dqacc_idx_0 - qw_idx_1 * dqacc_idx_1) -
                qw_idx_2 * dqacc_idx_2) -
               qw_idx_3 * dqacc_idx_3;
    qa_idx_1 = (qw_idx_0 * dqacc_idx_1 + dqacc_idx_0 * qw_idx_1) +
               (qw_idx_2 * dqacc_idx_3 - dqacc_idx_2 * qw_idx_3);
    qa_idx_2 = (qw_idx_0 * dqacc_idx_2 + dqacc_idx_0 * qw_idx_2) +
               (dqacc_idx_1 * qw_idx_3 - qw_idx_1 * dqacc_idx_3);
    n = (qw_idx_0 * dqacc_idx_3 + dqacc_idx_0 * qw_idx_3) +
        (qw_idx_1 * dqacc_idx_2 - dqacc_idx_1 * qw_idx_2);
    /* 2) Magnetometer-based correction */
    /* this function returns the rotation matrix based on the rotation
     * quaternion */
    /* the scalar part is located at the begin of the quaternion */

  
    bx = qa_idx_0 * qa_idx_0;
    by = qa_idx_1 * qa_idx_1;
    bz = qa_idx_2 * qa_idx_2;
    t = n * n;
    rot_matr[0] = ((bx + by) - bz) - t;
    rot_matr_tmp = qa_idx_1 * qa_idx_2;
    b_rot_matr_tmp = qa_idx_0 * n;
    rot_matr[1] = 2.0 * (rot_matr_tmp - b_rot_matr_tmp);
    c_rot_matr_tmp = qa_idx_1 * n;
    d_rot_matr_tmp = qa_idx_0 * qa_idx_2;
    rot_matr[2] = 2.0 * (c_rot_matr_tmp + d_rot_matr_tmp);
    rot_matr[3] = 2.0 * (rot_matr_tmp + b_rot_matr_tmp);
    bx -= by;
    rot_matr[4] = (bx + bz) - t;
    by = qa_idx_2 * n;
    rot_matr_tmp = qa_idx_0 * qa_idx_1;
    rot_matr[5] = 2.0 * (by - rot_matr_tmp);
    rot_matr[6] = 2.0 * (c_rot_matr_tmp - d_rot_matr_tmp);
    rot_matr[7] = 2.0 * (by + rot_matr_tmp);
    rot_matr[8] = (bx - bz) + t;
    for (i = 0; i < 3; i++) {
      gp[i] = (rot_matr[i] * hx + rot_matr[i + 3] * hy) + rot_matr[i + 6] * hz;
    }
    by = gp[0] * gp[0] + gp[1] * gp[1];
    t = by + gp[0] * sqrt(by);
    dqacc_idx_0 = sqrt(t) / sqrt(2.0 * by);
    dqacc_idx_3 = gp[1] / sqrt(2.0 * t);
    if (dqacc_idx_0 < 0.9) {
      /* if it is true compute the Spherical Interpolation */
      bz = acos(dqacc_idx_0);
      bx = sin(bz);
      t = sin((1.0 - gain_mag) * bz) / bx;
      bx = sin(gain_mag * bz) / bx;
      dqacc_idx_0 = t + bx * dqacc_idx_0;
      dqacc_idx_1 = t * 0.0 + bx * 0.0;
      dqacc_idx_2 = dqacc_idx_1;
      dqacc_idx_3 = t * 0.0 + bx * dqacc_idx_3;
    } else {
      bx = 3.3121686421112381E-170;
      if (dqacc_idx_0 > 3.3121686421112381E-170) {
        rot_matr_tmp = 1.0;
        bx = dqacc_idx_0;
      } else {
        rot_matr_tmp = 0;
      }
      bz = fabs(dqacc_idx_3);
      if (bz > bx) {
        t = bx / bz;
        rot_matr_tmp = rot_matr_tmp * t * t + 1.0;
        bx = bz;
      } else {
        t = bz / bx;
        rot_matr_tmp += t * t;
      }
      rot_matr_tmp = bx * sqrt(rot_matr_tmp);
      dqacc_idx_0 = ((1.0 - gain_mag) + gain_mag * dqacc_idx_0) / rot_matr_tmp;
      bx = (1.0 - gain_mag) * 0.0;
      dqacc_idx_1 = (bx + gain_mag * 0.0) / rot_matr_tmp;
      dqacc_idx_2 = dqacc_idx_1;
      dqacc_idx_3 = (bx + gain_mag * dqacc_idx_3) / rot_matr_tmp;
    }
    qvac[0] = ((qa_idx_0 * dqacc_idx_0 - qa_idx_1 * dqacc_idx_1) -
               qa_idx_2 * dqacc_idx_2) -
              n * dqacc_idx_3;
    qvac[1] = (qa_idx_0 * dqacc_idx_1 + dqacc_idx_0 * qa_idx_1) +
              (qa_idx_2 * dqacc_idx_3 - dqacc_idx_2 * n);
    qvac[2] = (qa_idx_0 * dqacc_idx_2 + dqacc_idx_0 * qa_idx_2) +
              (dqacc_idx_1 * n - qa_idx_1 * dqacc_idx_3);
    qvac[3] = (qa_idx_0 * dqacc_idx_3 + dqacc_idx_0 * n) +
              (qa_idx_1 * dqacc_idx_2 - dqacc_idx_1 * qa_idx_2);
    bx = sqrt(((qvac[0] * qvac[0] + qvac[1] * qvac[1]) + qvac[2] * qvac[2]) +
              qvac[3] * qvac[3]);
    qvac[0] /= bx;
    qvac[1] /= bx;
    qvac[2] /= bx;
    qvac[3] /= bx;
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */

int main()
{
 struct Euler euler;
 
  ax[0] = -0.003034;
  ax[1] = -0.001237;
  ax[2] =  0.002274;
  ax[3] = -0.008709;
  ax[4] = -0.008717;

  ay[0] = -0.026763;
  ay[1] = -0.023148;
  ay[2] = -0.003250;
  ay[3] =  0.004018;
  ay[4] = -0.006676;

  az[0] = 9.783478;
  az[1] = 9.785168;
  az[2] = 9.790104;
  az[3] = 9.783010;
  az[4] = 9.766892;

  wx[0] = 0.005243;
  wx[1] = 0.001595	;
  wx[2] = 0.005686	;
  wx[3] = -0.008085;
  wx[4] = 0.003135;

  wy[0] = -0.013765;
  wy[1] = -0.015912;
  wy[2] = -0.009041;
  wy[3] = -0.003802;
  wy[4] = 0.016505;

  wz[0] = 0.008987;
  wz[1] = 0.007902	;
  wz[2] = -0.014806	;
  wz[3] = -0.008875;
  wz[4] = 0.003565;

  hx[0] = 0.306959;
  hx[1]= 0.302215;
  hx[2] = 0.308222;
  hx[3] = 0.304595;
  hx[4] = 0.305587;

  hy[0] = -0.177699;
  hy[1] = -0.176207;
  hy[2] = -0.177541;
  hy[3] = -0.177255;
  hy[4] = -0.176824;

  hz[0] = -1.504173;
  hz[1] = -1.504083;
  hz[2] = -1.500194;
  hz[3] = -1.501145;
  hz[4] = -1.499498;



 for(int i = 0; i < 5; i++)
 {
 if(i == 0)
 {
        wx_= 0;
        wy_= 0;
        wz_= 0;
        qvac_[0] = 0; 
        qvac_[1] = 0; 
        qvac_[2] = 0;
        qvac_[3] = 1;
 }      
        else
        {
        wx_= wx[i-1];
        wy_= wy[i-1];
        wz_= wz[i-1];
        qvac_[0] = qvac[0]; 
        qvac_[1] = qvac[1]; 
        qvac_[2] = qvac[2];
        qvac_[3] = qvac[3];
        }
        VACimpl(qvac_,  ax[i], ay[i],  az[i], hx[i],
             hy[i], hz[i],  wx[i], wy[i],  wz[i],  wx_,
             wy_, wz_, gain_acc_static,  gain_mag,
              th1a,  th2a,  biasalpha,  qvac);

           
        calculateEuler(qvac, &euler);
    printf("Phi: %f, Theta: %f, Psi: %f\n", euler.phi*180./3.14, euler.theta*180./3.14, euler.psi*180./3.14);


 }
 
}
