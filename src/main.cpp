// Path Finder -- XR Hackathon 2026, Track 03
// Trajectory estimation under sensor degradation (IMU only).  C++17 port of
// main.py: same algorithm, same parameters, same output contract.
//
//     degraded EuRoC IMU CSV  ->  6-DoF trajectory in TUM format
//     (one pose per IMU frame, plus 5 s of post-failure dead reckoning)
//
// Usage:
//     pathfinder <imu0/data.csv | dataset_dir | dataset.zip> [-o estimated_trajectory.txt]
//                [--metadata metadata.json] [--no-extrapolate] [--set key=val ...]
//
// Build:
//     g++ -O2 -std=c++17 main.cpp -o pathfinder          (Linux / macOS / MinGW)
//     cl /O2 /EHsc /std:c++17 main.cpp /Fe:pathfinder.exe (MSVC)
//
// No third-party dependencies: .zip input is read with the built-in streaming
// inflate below (stored + deflate members), so the whole program streams with
// O(1) memory (~100 KB of buffers).  See README.md for the method.

#if defined(_MSC_VER) && !defined(_CRT_SECURE_NO_WARNINGS)
#define _CRT_SECURE_NO_WARNINGS
#endif
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <psapi.h>
#else
#include <sys/resource.h>
#endif

namespace fs = std::filesystem;

// ---------------------------------------------------------------------------
// Parameters (defaults identical to main.py DEFAULTS)
// ---------------------------------------------------------------------------
struct Params {
    double init_s = 1.0, kp = 0.0, kp_static = 2.0, ki = 0.0, tau_ba = 30.0, tau_aw = 5.0,
           tau_v = 1.0, tau_p = 5.0, tau_vb = 2.0, tau_wb = 0.1, gap_factor = 2.5,
           spike_k = 8.0, static_hold_s = 1e9, horizon_s = 5.0, shrink_gamma = 1.0,
           // rotor-drag velocity aiding (multirotors), gated by a multirotor detector
           drag = 1.0, drag_gain = -2.45, tau_n = 20.0, tau_c = 30.0, tau_fd = 0.3,
           tau_spread = 5.0, spread_lo = 0.14, spread_hi = 0.26, drag_warm = 15.0,
           // offline two-pass (zero-phase) smoothing
           smooth = 1.0, tau_vs = 4.0, tau_ps = 30.0, tau_vbs = 2.0;
    struct Entry { const char* name; double* v; };
    std::vector<Entry> table() {
        return {{"init_s", &init_s}, {"kp", &kp}, {"kp_static", &kp_static}, {"ki", &ki},
                {"tau_ba", &tau_ba}, {"tau_aw", &tau_aw}, {"tau_v", &tau_v}, {"tau_p", &tau_p},
                {"tau_vb", &tau_vb}, {"tau_wb", &tau_wb}, {"gap_factor", &gap_factor},
                {"spike_k", &spike_k}, {"static_hold_s", &static_hold_s},
                {"horizon_s", &horizon_s}, {"shrink_gamma", &shrink_gamma},
                {"drag", &drag}, {"drag_gain", &drag_gain}, {"tau_n", &tau_n}, {"tau_c", &tau_c},
                {"tau_fd", &tau_fd}, {"tau_spread", &tau_spread}, {"spread_lo", &spread_lo},
                {"spread_hi", &spread_hi}, {"drag_warm", &drag_warm},
                {"smooth", &smooth}, {"tau_vs", &tau_vs}, {"tau_ps", &tau_ps}, {"tau_vbs", &tau_vbs}};
    }
};

static const double GYRO_MAX = 40.0;   // rad/s
static const double ACC_MAX = 200.0;   // m/s^2

// ---------------------------------------------------------------------------
// Quaternion helpers (w, x, y, z), body -> world
// ---------------------------------------------------------------------------
struct Quat { double w, x, y, z; };
typedef std::array<std::array<double, 3>, 3> Mat3;
typedef std::array<double, 6> Meas;

static Quat q_mul(const Quat& a, const Quat& b) {
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

static Quat q_norm(const Quat& q) {
    double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    return {q.w / n, q.x / n, q.y / n, q.z / n};
}

static Quat q_integrate(const Quat& q, double wx, double wy, double wz, double dt) {
    double ang = std::sqrt(wx * wx + wy * wy + wz * wz) * dt;
    if (ang < 1e-12) return q;
    double h = 0.5 * ang;
    double s = std::sin(h) / (ang / dt);
    return q_norm(q_mul(q, {std::cos(h), wx * s, wy * s, wz * s}));
}

static Mat3 q_to_rot(const Quat& q) {
    double w = q.w, x = q.x, y = q.y, z = q.z;
    double xx = x * x, yy = y * y, zz = z * z;
    double xy = x * y, xz = x * z, yz = y * z;
    double wx = w * x, wy = w * y, wz = w * z;
    Mat3 R;
    R[0] = {1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)};
    R[1] = {2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)};
    R[2] = {2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)};
    return R;
}

static Quat q_from_two_vectors(const double u[3], const double v[3]) {
    double d = u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
    double cx = u[1] * v[2] - u[2] * v[1];
    double cy = u[2] * v[0] - u[0] * v[2];
    double cz = u[0] * v[1] - u[1] * v[0];
    if (d < -0.999999) {
        double ax[3] = {1.0, 0.0, 0.0};
        if (!(std::fabs(u[0]) < 0.9)) { ax[0] = 0.0; ax[1] = 1.0; }
        cx = u[1] * ax[2] - u[2] * ax[1];
        cy = u[2] * ax[0] - u[0] * ax[2];
        cz = u[0] * ax[1] - u[1] * ax[0];
        return q_norm({0.0, cx, cy, cz});
    }
    return q_norm({1.0 + d, cx, cy, cz});
}

static inline double mn1(double x) { return x < 1.0 ? x : 1.0; }   // Python min(1.0, x)

// ---------------------------------------------------------------------------
// Offline smoother records (one per filter step and per output row)
// ---------------------------------------------------------------------------
enum { REC_STEP = 0, REC_BLIND = 1, REC_ROW = 2 };
struct Rec {
    int32_t kind, drag, st, pad;
    int64_t ts;
    double d[12];   // step: dt, a(3), vt(3) | row: q(x,y,z,w), beta, raw pos(3)
};

#ifdef _WIN32
#define FSEEK64 _fseeki64
#define FTELL64 _ftelli64
#else
#define FSEEK64 fseeko
#define FTELL64 ftello
#endif

class RecWriter {
public:
    explicit RecWriter(FILE* f) : f(f) { buf.reserve(1024); }
    long long nsteps = 0, klast = -1;   // steps so far, index of last measured step
    void push(const Rec& r) {
        buf.push_back(r);
        if (buf.size() >= 1024) flush();
    }
    void step(int kind, int drag, int st, double dt, double ax, double ay, double az,
              double vx, double vy, double vz) {
        if (kind == REC_STEP) klast = nsteps;
        ++nsteps;
        Rec r{};
        r.kind = kind; r.drag = drag; r.st = st;
        r.d[0] = dt; r.d[1] = ax; r.d[2] = ay; r.d[3] = az; r.d[4] = vx; r.d[5] = vy; r.d[6] = vz;
        push(r);
    }
    void flush() {
        if (!buf.empty()) fwrite(buf.data(), sizeof(Rec), buf.size(), f);
        buf.clear();
    }

private:
    FILE* f;
    std::vector<Rec> buf;
};

// ---------------------------------------------------------------------------
// Estimator (mirrors class PathFinder in main.py)
// ---------------------------------------------------------------------------
class PathFinder {
public:
    explicit PathFinder(const Params& p) : p(p) {}

    Params p;
    Quat q{1.0, 0.0, 0.0, 0.0};
    double pos[3] = {0, 0, 0}, v[3] = {0, 0, 0}, vs[3] = {0, 0, 0}, ws[3] = {0, 0, 0};
    double bg[3] = {0, 0, 0}, ba[3] = {0, 0, 0}, mw[3] = {0, 0, 0};
    double g = 9.81;
    bool has_t_prev = false;
    int64_t t_prev = 0;
    double dt_nom = 0.005;
    bool has_last_raw = false;
    Meas last_raw{};
    int stale = 0;
    bool has_ref = false;
    Meas ref{};
    double dn[6] = {0, 0, 0, 0, 0, 0};
    int rej[6] = {0, 0, 0, 0, 0, 0};
    double nw = 0.0, na = 0.0, na_min = 1e9, t_run = 0.0, beta = 1.0;
    double ew = 1.0, ea = 1.0, static_t = 0.0;
    double tb = 0.0, v_hold[3] = {0, 0, 0}, w_hold[3] = {0, 0, 0};
    long long n_invalid = 0, n_spikes = 0, n_gaps = 0;
    double blind_time = 0.0;
    bool init_static = false;
    RecWriter* sink = nullptr;          // step recorder for the offline smoother
    // rotor-drag aiding state (IMU frame)
    double fbar[3] = {0.0, 0.0, 9.81};
    int eref = 0;
    double lc[2] = {0.0, 0.0}, fd[2] = {0.0, 0.0};
    double spread = 0.0, dgate = 0.0, t_drag = 0.0, drag_gate_time = 0.0;

    void initialise(const std::vector<Meas>& all, double dtn) {
        dt_nom = dtn;
        if (all.empty()) return;
        // robust to impulsive spikes: drop rows far from the per-channel median
        // (one gyro spike in the first second would otherwise void the static test)
        std::vector<Meas> samples = all;
        {
            size_t n0 = all.size();
            double med[6];
            std::vector<double> col(n0);
            for (int i = 0; i < 6; ++i) {
                for (size_t r = 0; r < n0; ++r) col[r] = all[r][i];
                std::sort(col.begin(), col.end());
                med[i] = col[n0 / 2];
            }
            std::vector<Meas> good;
            for (const Meas& m : all) {
                bool ok = true;
                for (int i = 0; i < 6 && ok; ++i)
                    if (std::fabs(m[i] - med[i]) > (i < 3 ? 0.5 : 3.0)) ok = false;
                if (ok) good.push_back(m);
            }
            if (good.size() >= std::max<size_t>(10, n0 / 2)) samples.swap(good);
        }
        double n = (double)samples.size();
        double mf[3], mwv[3];
        for (int i = 0; i < 3; ++i) {
            double s = 0.0;
            for (const Meas& m : samples) s += m[3 + i];
            mf[i] = s / n;
        }
        for (int i = 0; i < 3; ++i) {
            double s = 0.0;
            for (const Meas& m : samples) s += m[i];
            mwv[i] = s / n;
        }
        double vw = 0.0;
        for (const Meas& m : samples)
            for (int i = 0; i < 3; ++i) vw += (m[i] - mwv[i]) * (m[i] - mwv[i]);
        vw /= n;
        std::vector<double> fn;
        fn.reserve(samples.size());
        for (const Meas& m : samples) fn.push_back(std::sqrt(m[3] * m[3] + m[4] * m[4] + m[5] * m[5]));
        double mfn = 0.0;
        for (double x : fn) mfn += x;
        mfn /= n;
        double vfn = 0.0;
        for (double x : fn) vfn += (x - mfn) * (x - mfn);
        vfn /= n;
        double norm = std::sqrt(mf[0] * mf[0] + mf[1] * mf[1] + mf[2] * mf[2]);
        if (norm < 1e-6) return;
        bool st = vw < 0.05 * 0.05 && vfn < 0.3 * 0.3 && std::fabs(norm - 9.81) < 1.0;
        if (st) {
            for (int i = 0; i < 3; ++i) bg[i] = mwv[i];
            g = norm;
            static_t = p.static_hold_s;
        }
        double u[3] = {mf[0] / norm, mf[1] / norm, mf[2] / norm};
        double up[3] = {0.0, 0.0, 1.0};
        q = q_from_two_vectors(u, up);
        init_static = st;
        for (int i = 0; i < 3; ++i) fbar[i] = mf[i];
        double ab[3] = {std::fabs(mf[0]), std::fabs(mf[1]), std::fabs(mf[2])};
        eref = 0;
        for (int i = 1; i < 3; ++i)
            if (ab[i] < ab[eref]) eref = i;
    }

    // m == nullptr means "no valid measurement for this frame"
    void update(int64_t ts, const Meas* m) {
        if (!has_t_prev) {
            has_t_prev = true;
            t_prev = ts;
            if (m) { ref = *m; has_ref = true; }
            return;
        }
        double dt = (double)(ts - t_prev) * 1e-9;
        if (dt <= 0.0) return;
        t_prev = ts;
        double dtn = dt_nom;
        if (dt > p.gap_factor * dtn) {
            ++n_gaps;
            double gap = dt - dtn;
            long long k = (long long)(gap / dtn + 0.5);
            if (k > 20000) k = 20000;
            if (k > 0) {
                double sub = gap / (double)k;
                for (long long i = 0; i < k; ++i) blind(sub);
            }
            dt = dtn;
        }
        Meas out;
        if (!gate(m, dt, out)) {
            ++n_invalid;
            blind(dt);
        } else {
            tb = 0.0;
            propagate(dt, out);
        }
    }

    void pose(double P[3], double Q[4]) {
        double w = q.w, x = q.x, y = q.y, z = q.z;
        if (w < 0.0) { w = -w; x = -x; y = -y; z = -z; }
        double gam = p.shrink_gamma;
        if (gam > 0.0 && na > 0.0 && na_min < 1e8) {
            double b = mn1(std::pow(na_min / na, 0.5 * gam));
            beta += 0.01 * (b - beta);
        }
        for (int i = 0; i < 3; ++i) P[i] = pos[i] * beta;
        Q[0] = x; Q[1] = y; Q[2] = z; Q[3] = w;
    }

private:
    bool gate(const Meas* mp, double dt, Meas& out) {
        if (!mp) return false;
        const Meas& m = *mp;
        if (has_last_raw && m == last_raw) {
            ++stale;
        } else {
            stale = 0;
            last_raw = m;
            has_last_raw = true;
        }
        if (stale >= 3) return false;
        if (m[3] == 0.0 && m[4] == 0.0 && m[5] == 0.0) return false;
        for (int i = 0; i < 3; ++i)
            if (std::fabs(m[i]) > GYRO_MAX || std::fabs(m[3 + i]) > ACC_MAX) return false;
        if (!has_ref) {
            ref = m;
            has_ref = true;
            out = m;
            return true;
        }
        if (tb > 0.0) {
            ref = m;
            for (int i = 0; i < 6; ++i) rej[i] = 0;
            out = m;
            return true;
        }
        double k = p.spike_k;
        double a = mn1(dt / 0.5);
        out = m;
        for (int i = 0; i < 6; ++i) {
            double d = m[i] - ref[i];
            double floor_ = i < 3 ? 0.5 : 3.0;
            if (std::fabs(d) > k * dn[i] + floor_ && rej[i] < 2) {
                out[i] = ref[i];
                ++rej[i];
                ++n_spikes;
            } else {
                if (rej[i] == 0) dn[i] += a * (std::fabs(d) - dn[i]);
                rej[i] = 0;
                ref[i] = m[i];
            }
        }
        return true;
    }

    void propagate(double dt, const Meas& m) {
        double wx = m[0] - bg[0], wy = m[1] - bg[1], wz = m[2] - bg[2];
        double fx = m[3], fy = m[4], fz = m[5];

        double a_n = mn1(dt / 2.0);
        double a_s = mn1(dt / 0.2);
        nw += a_n * ((dn[0] * dn[0] + dn[1] * dn[1] + dn[2] * dn[2]) - nw);
        na += a_n * ((dn[3] * dn[3] + dn[4] * dn[4] + dn[5] * dn[5]) - na);
        t_run += dt;
        if (t_run > 3.0 && na < na_min) na_min = na;
        double fn = std::sqrt(fx * fx + fy * fy + fz * fz);
        ew += a_s * ((wx * wx + wy * wy + wz * wz) - ew);
        ea += a_s * ((fn - g) * (fn - g) - ea);
        bool still = ew < 4.0 * nw + 0.02 * 0.02 && ea < 1.5 * na + 0.15 * 0.15;
        static_t = still ? static_t + dt : 0.0;
        bool is_static = static_t >= p.static_hold_s;

        Mat3 R = q_to_rot(q);
        double vx = R[2][0], vy = R[2][1], vz = R[2][2];
        if (fn > 1e-6) {
            double ux = fx / fn, uy = fy / fn, uz = fz / fn;
            double r = (fn - g) / (0.15 * g);
            double trust = std::exp(-r * r);
            double ex = uy * vz - uz * vy;
            double ey = uz * vx - ux * vz;
            double ez = ux * vy - uy * vx;
            double kp = (is_static ? p.kp_static : p.kp) * trust;
            double ki = p.ki * trust * dt;
            wx += kp * ex; wy += kp * ey; wz += kp * ez;
            bg[0] -= ki * ex; bg[1] -= ki * ey; bg[2] -= ki * ez;
        }
        if (is_static) {
            double b = mn1(dt / 1.0);
            for (int i = 0; i < 3; ++i) bg[i] += b * (m[i] - bg[i]);
        }
        q = q_integrate(q, wx, wy, wz, dt);
        double b = mn1(dt / 0.1);
        ws[0] += b * (wx - ws[0]);
        ws[1] += b * (wy - ws[1]);
        ws[2] += b * (wz - ws[2]);

        R = q_to_rot(q);
        vx = R[2][0]; vy = R[2][1]; vz = R[2][2];
        double tba = is_static ? 0.5 : p.tau_ba;
        b = mn1(dt / tba);
        ba[0] += b * (fx - g * vx - ba[0]);
        ba[1] += b * (fy - g * vy - ba[1]);
        ba[2] += b * (fz - g * vz - ba[2]);
        double bx = fx - ba[0], by = fy - ba[1], bz = fz - ba[2];
        double ax = R[0][0] * bx + R[0][1] * by + R[0][2] * bz;
        double ay = R[1][0] * bx + R[1][1] * by + R[1][2] * bz;
        double az = R[2][0] * bx + R[2][1] * by + R[2][2] * bz - g;
        b = mn1(dt / p.tau_aw);
        mw[0] += b * (ax - mw[0]);
        mw[1] += b * (ay - mw[1]);
        mw[2] += b * (az - mw[2]);
        ax -= mw[0]; ay -= mw[1]; az -= mw[2];

        // --- rotor-drag velocity aiding (see main.py for the rationale) ---
        bool drag_on = false;
        double vtx = 0.0, vty = 0.0, vtz = 0.0;
        if (p.drag > 0.0 && !is_static) {
            b = mn1(dt / p.tau_n);
            fbar[0] += b * (fx - fbar[0]);
            fbar[1] += b * (fy - fbar[1]);
            fbar[2] += b * (fz - fbar[2]);
            double nn = std::sqrt(fbar[0] * fbar[0] + fbar[1] * fbar[1] + fbar[2] * fbar[2]);
            if (nn > 1e-6 && fn > 1e-6) {
                double nx = fbar[0] / nn, ny = fbar[1] / nn, nz = fbar[2] / nn;
                double cs = (fx * nx + fy * ny + fz * nz) / fn;
                b = mn1(dt / p.tau_spread);
                spread += b * (2.0 * (1.0 - cs) - spread);
                double ex_ = eref == 0 ? 1.0 : 0.0, ey_ = eref == 1 ? 1.0 : 0.0, ez_ = eref == 2 ? 1.0 : 0.0;
                double c1x = ny * ez_ - nz * ey_;
                double c1y = nz * ex_ - nx * ez_;
                double c1z = nx * ey_ - ny * ex_;
                double cn = std::sqrt(c1x * c1x + c1y * c1y + c1z * c1z);
                c1x = c1x / cn; c1y = c1y / cn; c1z = c1z / cn;
                double c2x = ny * c1z - nz * c1y;
                double c2y = nz * c1x - nx * c1z;
                double c2z = nx * c1y - ny * c1x;
                double l1 = fx * c1x + fy * c1y + fz * c1z;
                double l2 = fx * c2x + fy * c2y + fz * c2z;
                b = mn1(dt / p.tau_c);
                lc[0] += b * (l1 - lc[0]);
                lc[1] += b * (l2 - lc[1]);
                b = mn1(dt / p.tau_fd);
                fd[0] += b * ((l1 - lc[0]) - fd[0]);
                fd[1] += b * ((l2 - lc[1]) - fd[1]);
                t_drag += dt;
                double ang = spread > 0.0 ? std::sqrt(spread) : 0.0;
                double gk = (p.spread_hi - ang) / (p.spread_hi - p.spread_lo);
                double gt = gk < 0.0 ? 0.0 : (gk > 1.0 ? 1.0 : gk);
                if (t_drag < p.drag_warm) gt = 0.0;
                dgate = gt;
                if (gt > 0.0) {
                    double kg = gt * p.drag_gain;
                    double s1 = kg * fd[0];
                    double s2 = kg * fd[1];
                    double tbx = s1 * c1x + s2 * c2x;
                    double tby = s1 * c1y + s2 * c2y;
                    double tbz = s1 * c1z + s2 * c2z;
                    vtx = R[0][0] * tbx + R[0][1] * tby + R[0][2] * tbz;
                    vty = R[1][0] * tbx + R[1][1] * tby + R[1][2] * tbz;
                    vtz = R[2][0] * tbx + R[2][1] * tby + R[2][2] * tbz;
                    drag_on = true;
                    drag_gate_time += dt * gt;
                }
            }
        }

        if (sink) sink->step(REC_STEP, drag_on ? 1 : 0, is_static ? 1 : 0, dt, ax, ay, az, vtx, vty, vtz);

        double ox = v[0], oy = v[1], oz = v[2];
        if (is_static) {
            double dec = std::exp(-dt / 0.05);
            v[0] *= dec; v[1] *= dec; v[2] *= dec;
        } else if (drag_on) {
            double dec = 1.0 - dt / p.tau_v;
            double kv = dt / p.tau_v;
            v[0] = (v[0] + ax * dt) * dec + vtx * kv;
            v[1] = (v[1] + ay * dt) * dec + vty * kv;
            v[2] = (v[2] + az * dt) * dec + vtz * kv;
        } else {
            double dec = 1.0 - dt / p.tau_v;
            v[0] = (v[0] + ax * dt) * dec;
            v[1] = (v[1] + ay * dt) * dec;
            v[2] = (v[2] + az * dt) * dec;
        }
        double lp = 1.0 - dt / p.tau_p;
        double h = 0.5 * dt;
        pos[0] = (pos[0] + h * (ox + v[0])) * lp;
        pos[1] = (pos[1] + h * (oy + v[1])) * lp;
        pos[2] = (pos[2] + h * (oz + v[2])) * lp;
        b = mn1(dt / 0.2);
        vs[0] += b * (v[0] - vs[0]);
        vs[1] += b * (v[1] - vs[1]);
        vs[2] += b * (v[2] - vs[2]);
    }

    void blind(double dt) {
        if (sink) sink->step(REC_BLIND, 0, 0, dt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        if (tb == 0.0) {
            for (int i = 0; i < 3; ++i) { v_hold[i] = vs[i]; w_hold[i] = ws[i]; }
        }
        tb += dt;
        blind_time += dt;
        double dw = std::exp(-tb / p.tau_wb);
        q = q_integrate(q, w_hold[0] * dw, w_hold[1] * dw, w_hold[2] * dw, dt);
        double dv = std::exp(-tb / p.tau_vb);
        for (int i = 0; i < 3; ++i) {
            double nv = v_hold[i] * dv;
            pos[i] += 0.5 * dt * (v[i] + nv);
            v[i] = nv;
        }
    }
};

// ---------------------------------------------------------------------------
// Row parsing (mirrors parse_rows in main.py)
// ---------------------------------------------------------------------------
static inline bool is_ws(char c) { return c == ' ' || c == '\t' || c == '\v' || c == '\f' || c == '\r' || c == '\n'; }

static void trim(const char*& b, const char*& e) {
    while (b < e && is_ws(*b)) ++b;
    while (e > b && is_ws(e[-1])) --e;
}

// Python float(): whole (whitespace-trimmed) field must parse.
static bool parse_double(const char* b, const char* e, double& out) {
    trim(b, e);
    if (b == e) return false;
    char buf[96];
    size_t n = (size_t)(e - b);
    if (n >= sizeof(buf)) return false;
    memcpy(buf, b, n);
    buf[n] = 0;
    if (n > 1 && buf[0] == '0' && (buf[1] == 'x' || buf[1] == 'X')) return false;  // no hex in Python
    char* end = nullptr;
    out = std::strtod(buf, &end);
    return end == buf + n;
}

static bool parse_ts(const char* b, const char* e, int64_t& ts) {
    trim(b, e);
    if (b == e) return false;
    bool digits = true;
    for (const char* c = b; c < e; ++c)
        if (*c < '0' || *c > '9') { digits = false; break; }
    if (digits) {                               // exact integer parse (19-digit ns stamps)
        uint64_t v = 0;
        for (const char* c = b; c < e; ++c) {
            unsigned d = (unsigned)(*c - '0');
            if (v > (uint64_t)(INT64_MAX - d) / 10) return false;   // would overflow int64
            v = v * 10 + d;
        }
        ts = (int64_t)v;
        return true;
    }
    double d;
    if (!parse_double(b, e, d) || !std::isfinite(d)) return false;
    ts = (int64_t)d;                            // int(float(t)) truncates toward zero
    return true;
}

// returns false if the line is skipped; valid=false if the measurement is unusable
static bool parse_line(const char* b, const char* e, int64_t& ts, Meas& m, bool& valid) {
    trim(b, e);
    if (b == e || *b == '#') return false;
    const char* fields[8];
    const char* fend[8];
    int nf = 0;
    const char* s = b;
    for (const char* c = b;; ++c) {
        if (c == e || *c == ',') {
            if (nf < 8) { fields[nf] = s; fend[nf] = c; }
            ++nf;
            s = c + 1;
            if (c == e) break;
        }
    }
    if (!parse_ts(fields[0], fend[0], ts)) return false;
    valid = false;
    if (nf >= 7) {
        valid = true;
        for (int i = 0; i < 6; ++i) {
            double x;
            if (!parse_double(fields[1 + i], fend[1 + i], x) || !std::isfinite(x)) { valid = false; break; }
            m[i] = x;
        }
    }
    return true;
}

static std::string fmt_ts(int64_t ts) {
    char buf[48];
    const char* sign = ts < 0 ? "-" : "";
    uint64_t a = ts < 0 ? (uint64_t)(-(ts + 1)) + 1 : (uint64_t)ts;
    snprintf(buf, sizeof buf, "%s%llu.%09llu", sign, (unsigned long long)(a / 1000000000ULL),
             (unsigned long long)(a % 1000000000ULL));
    return buf;
}

// ---------------------------------------------------------------------------
// Offline two-pass smoother (mirrors _smooth in main.py)
// ---------------------------------------------------------------------------
template <class T>
class FwdReader {
public:
    explicit FwdReader(FILE* f) : f(f), buf(1024) { FSEEK64(f, 0, SEEK_SET); }
    bool next(T& x) {
        if (i == n) {
            n = fread(buf.data(), sizeof(T), buf.size(), f);
            i = 0;
            if (n == 0) return false;
        }
        x = buf[i++];
        return true;
    }

private:
    FILE* f;
    std::vector<T> buf;
    size_t i = 0, n = 0;
};

template <class T>
class BwdReader {
public:
    explicit BwdReader(FILE* f) : f(f), buf(1024) {
        FSEEK64(f, 0, SEEK_END);
        end = (long long)FTELL64(f) / (long long)sizeof(T);
    }
    bool next(T& x) {
        if (i == 0) {
            if (end <= 0) return false;
            long long start = end - (long long)buf.size();
            if (start < 0) start = 0;
            FSEEK64(f, start * (long long)sizeof(T), SEEK_SET);
            i = fread(buf.data(), sizeof(T), (size_t)(end - start), f);
            end = start;
            if (i == 0) return false;
        }
        x = buf[--i];
        return true;
    }

private:
    FILE* f;
    std::vector<T> buf;
    long long end = 0;
    size_t i = 0;
};

struct V3r { double v[3]; };
struct V6r { double v[6]; };

static void vstep(double v[3], const Rec& r, double sgn, const Params& p) {
    double dt = r.d[0];
    if (r.st) {
        double d = std::exp(-dt / 0.05);
        v[0] *= d; v[1] *= d; v[2] *= d;
        return;
    }
    if (r.kind == REC_BLIND) {
        double d = std::exp(-dt / p.tau_vbs);
        v[0] *= d; v[1] *= d; v[2] *= d;
        return;
    }
    double dec = 1.0 - dt / p.tau_vs;
    if (r.drag) {
        double kv = dt / p.tau_vs;
        v[0] = (v[0] + sgn * r.d[1] * dt) * dec + r.d[4] * kv;
        v[1] = (v[1] + sgn * r.d[2] * dt) * dec + r.d[5] * kv;
        v[2] = (v[2] + sgn * r.d[3] * dt) * dec + r.d[6] * kv;
    } else {
        v[0] = (v[0] + sgn * r.d[1] * dt) * dec;
        v[1] = (v[1] + sgn * r.d[2] * dt) * dec;
        v[2] = (v[2] + sgn * r.d[3] * dt) * dec;
    }
}

static void smooth_passes(const fs::path& dir, const fs::path& t1p, FILE* out, const Params& p,
                          long long k_last) {
    const double tps = p.tau_ps;
    fs::path t2p = dir / "vf.bin", t3p = dir / "vb.bin";
    FILE* f1 = fopen(t1p.string().c_str(), "rb");
    if (!f1) throw std::runtime_error("smoother: cannot reopen temp file");
    {   // B1: forward velocity
        FILE* f2 = fopen(t2p.string().c_str(), "wb");
        if (!f2) throw std::runtime_error("smoother: cannot create temp file");
        std::vector<V3r> wb;
        wb.reserve(1024);
        double v[3] = {0.0, 0.0, 0.0};
        FwdReader<Rec> rd(f1);
        Rec r;
        while (rd.next(r)) {
            if (r.kind == REC_ROW) continue;
            vstep(v, r, 1.0, p);
            wb.push_back({{v[0], v[1], v[2]}});
            if (wb.size() >= 1024) { fwrite(wb.data(), sizeof(V3r), wb.size(), f2); wb.clear(); }
        }
        if (!wb.empty()) fwrite(wb.data(), sizeof(V3r), wb.size(), f2);
        fclose(f2);
    }
    {   // B2: backward velocity, averaged velocity, backward position
        FILE* f2 = fopen(t2p.string().c_str(), "rb");
        FILE* f3 = fopen(t3p.string().c_str(), "wb");
        if (!f2 || !f3) throw std::runtime_error("smoother: temp file error");
        BwdReader<Rec> rd(f1);
        BwdReader<V3r> itf(f2);
        std::vector<V6r> wb;
        wb.reserve(1024);
        double vb[3] = {0.0, 0.0, 0.0}, pb[3] = {0.0, 0.0, 0.0};
        Rec r;
        V3r vf;
        while (rd.next(r)) {
            if (r.kind == REC_ROW) continue;
            if (!itf.next(vf)) throw std::runtime_error("smoother: record mismatch");
            vstep(vb, r, -1.0, p);
            double dt = r.d[0];
            double lp = 1.0 - dt / tps;
            double vx = 0.5 * (vf.v[0] + vb[0]);
            double vy = 0.5 * (vf.v[1] + vb[1]);
            double vz = 0.5 * (vf.v[2] + vb[2]);
            pb[0] = (pb[0] - vx * dt) * lp;
            pb[1] = (pb[1] - vy * dt) * lp;
            pb[2] = (pb[2] - vz * dt) * lp;
            wb.push_back({{vx, vy, vz, pb[0], pb[1], pb[2]}});
            if (wb.size() >= 1024) { fwrite(wb.data(), sizeof(V6r), wb.size(), f3); wb.clear(); }
        }
        if (!wb.empty()) fwrite(wb.data(), sizeof(V6r), wb.size(), f3);
        fclose(f2);
        fclose(f3);
    }
    {   // C: forward position, average, write poses
        FILE* f3 = fopen(t3p.string().c_str(), "rb");
        if (!f3) throw std::runtime_error("smoother: temp file error");
        FwdReader<Rec> rd(f1);
        BwdReader<V6r> itb(f3);
        double pf[3] = {0.0, 0.0, 0.0}, ps[3] = {0.0, 0.0, 0.0};
        double ps_fail[3] = {0.0, 0.0, 0.0}, pos_ref[3] = {0.0, 0.0, 0.0};
        long long count = 0;
        std::string obuf;
        obuf.reserve(1 << 16);
        Rec r;
        V6r b;
        while (rd.next(r)) {
            if (r.kind != REC_ROW) {
                if (!itb.next(b)) throw std::runtime_error("smoother: record mismatch");
                double dt = r.d[0];
                double lp = 1.0 - dt / tps;
                pf[0] = (pf[0] + b.v[0] * dt) * lp;
                pf[1] = (pf[1] + b.v[1] * dt) * lp;
                pf[2] = (pf[2] + b.v[2] * dt) * lp;
                ps[0] = 0.5 * (pf[0] + b.v[3]);
                ps[1] = 0.5 * (pf[1] + b.v[4]);
                ps[2] = 0.5 * (pf[2] + b.v[5]);
                if (count == k_last) { ps_fail[0] = ps[0]; ps_fail[1] = ps[1]; ps_fail[2] = ps[2]; }
                ++count;
                continue;
            }
            double beta = r.d[4];
            double px, py, pz;
            if (count - 1 <= k_last) {
                px = ps[0]; py = ps[1]; pz = ps[2];
                pos_ref[0] = r.d[5]; pos_ref[1] = r.d[6]; pos_ref[2] = r.d[7];
            } else {
                px = ps_fail[0] + r.d[5] - pos_ref[0];
                py = ps_fail[1] + r.d[6] - pos_ref[1];
                pz = ps_fail[2] + r.d[7] - pos_ref[2];
            }
            char line[256];
            int n = snprintf(line, sizeof line, "%s %.6f %.6f %.6f %.9f %.9f %.9f %.9f\n",
                             fmt_ts(r.ts).c_str(), px * beta, py * beta, pz * beta,
                             r.d[0], r.d[1], r.d[2], r.d[3]);
            obuf.append(line, (size_t)n);
            if (obuf.size() >= (1 << 16) - 256) { fwrite(obuf.data(), 1, obuf.size(), out); obuf.clear(); }
        }
        if (!obuf.empty()) fwrite(obuf.data(), 1, obuf.size(), out);
        fclose(f3);
    }
    fclose(f1);
}

// ---------------------------------------------------------------------------
// Pipeline: push-based (bytes -> lines -> rows -> poses)
// ---------------------------------------------------------------------------
class Pipeline {
public:
    Pipeline(const Params& p, FILE* out, bool extrapolate)
        : prm(p), est(p), out(out), extrapolate(extrapolate) {
        obuf.reserve(1 << 16);
        line.reserve(256);
        if (prm.smooth > 0.0) {
            std::error_code ec;
            char name[64];
            snprintf(name, sizeof name, "pathfinder_%llx_%llx",
                     (unsigned long long)std::chrono::steady_clock::now().time_since_epoch().count(),
                     (unsigned long long)(uintptr_t)this);
            tmpdir = fs::temp_directory_path(ec) / name;
            fs::create_directories(tmpdir, ec);
            t1path = tmpdir / "steps.bin";
            t1 = fopen(t1path.string().c_str(), "wb");
            if (!t1) {   // no usable temp space: fall back to the causal single pass
                fprintf(stderr, "[pathfinder] warning: cannot create temp file in %s; "
                                "using causal (single-pass) mode\n", tmpdir.string().c_str());
                prm.smooth = 0.0;
                fs::remove_all(tmpdir, ec);
                tmpdir.clear();
            } else {
                rec = new RecWriter(t1);
                est.sink = rec;
            }
        }
    }
    ~Pipeline() {
        if (rec) { delete rec; rec = nullptr; }
        if (t1) { fclose(t1); t1 = nullptr; }
        if (!tmpdir.empty()) { std::error_code ec; fs::remove_all(tmpdir, ec); }
    }

    // ---- byte sink ----
    void put(char c) {
        if (c == '\n' || c == '\r') {
            if (!line.empty()) { on_line(); line.clear(); }
        } else {
            line.push_back(c);
        }
    }
    void put(const char* p, size_t n) { for (size_t i = 0; i < n; ++i) put(p[i]); }

    void finish() {
        if (!line.empty()) { on_line(); line.clear(); }
        if (in_head) start();
        if (head_rows == 0) throw std::runtime_error("no IMU rows found");
        if (extrapolate && has_last && (double)trailing_invalid_ns < 0.8 * prm.horizon_s * 1e9) {
            n_extra = (long long)std::nearbyint(prm.horizon_s * 1e9 / (double)dt_nom_ns);
            for (long long k = 1; k <= n_extra; ++k) {
                int64_t ts = last_ts + k * dt_nom_ns;
                est.update(ts, nullptr);
                emit(ts);
            }
        }
        flush();
        if (rec) {
            rec->flush();
            fclose(t1);
            t1 = nullptr;
            smooth_passes(tmpdir, t1path, out, prm, rec->klast);
        }
    }

    Params prm;
    PathFinder est;
    long long n_out = 0, n_extra = 0, head_rows = 0;
    int64_t t_first = 0, last_ts = 0, dt_nom_ns = 5000000;
    bool has_last = false;

private:
    struct Row { int64_t ts; Meas m; bool valid; };
    FILE* out;
    bool extrapolate;
    std::string line, obuf;
    bool in_head = true, has_first = false;
    std::vector<Row> head;
    int64_t trailing_invalid_ns = 0, run_start = 0;
    bool has_run_start = false;
    fs::path tmpdir, t1path;
    FILE* t1 = nullptr;
    RecWriter* rec = nullptr;

    void on_line() {
        int64_t ts;
        Meas m{};
        bool valid;
        if (!parse_line(line.data(), line.data() + line.size(), ts, m, valid)) return;
        if (in_head) {
            if (!has_first) { has_first = true; t_first = ts; }
            head.push_back({ts, m, valid});
            ++head_rows;
            if ((double)(ts - t_first) >= prm.init_s * 1e9 && head.size() >= 20) start();
            return;
        }
        consume(ts, valid ? &m : nullptr);
    }

    void start() {
        in_head = false;
        if (head.empty()) return;
        std::vector<int64_t> diffs;
        for (size_t i = 0; i + 1 < head.size(); ++i)
            if (head[i + 1].ts > head[i].ts) diffs.push_back(head[i + 1].ts - head[i].ts);
        std::sort(diffs.begin(), diffs.end());
        dt_nom_ns = diffs.empty() ? 5000000 : diffs[diffs.size() / 2];
        std::vector<Meas> valid;
        for (const Row& r : head)
            if (r.valid) valid.push_back(r.m);
        est.initialise(valid, (double)dt_nom_ns * 1e-9);
        std::vector<Row> h;
        h.swap(head);
        for (const Row& r : h) consume(r.ts, r.valid ? &r.m : nullptr);
    }

    void consume(int64_t ts, const Meas* m) {
        est.update(ts, m);
        emit(ts);
        ++n_out;
        if (!m) {
            if (!has_run_start) {
                has_run_start = true;
                run_start = has_last ? last_ts : ts;
            }
            trailing_invalid_ns = ts - run_start;
        } else {
            has_run_start = false;
            trailing_invalid_ns = 0;
        }
        last_ts = ts;
        has_last = true;
    }

    void emit(int64_t ts) {
        double P[3], Q[4];
        est.pose(P, Q);
        if (rec) {
            Rec r{};
            r.kind = REC_ROW;
            r.ts = ts;
            r.d[0] = Q[0]; r.d[1] = Q[1]; r.d[2] = Q[2]; r.d[3] = Q[3];
            r.d[4] = est.beta;
            r.d[5] = est.pos[0]; r.d[6] = est.pos[1]; r.d[7] = est.pos[2];
            rec->push(r);
            return;
        }
        char buf[256];
        int n = snprintf(buf, sizeof buf, "%s %.6f %.6f %.6f %.9f %.9f %.9f %.9f\n",
                         fmt_ts(ts).c_str(), P[0], P[1], P[2], Q[0], Q[1], Q[2], Q[3]);
        obuf.append(buf, (size_t)n);
        if (obuf.size() >= (1 << 16) - 256) flush();
    }

    void flush() {
        if (!obuf.empty()) fwrite(obuf.data(), 1, obuf.size(), out);
        obuf.clear();
    }
};

// ---------------------------------------------------------------------------
// Minimal streaming inflate (RFC 1951), after Mark Adler's puff.c, with a
// 32 KB sliding window so output is pushed straight into the pipeline.
// ---------------------------------------------------------------------------
class Inflater {
public:
    Inflater(FILE* f, Pipeline& sink) : f(f), sink(sink) {}

    void run() {
        int last;
        do {
            last = bits(1);
            int type = bits(2);
            if (type == 0) stored();
            else if (type == 1) fixed();
            else if (type == 2) dynamic();
            else throw std::runtime_error("invalid deflate block type");
        } while (!last);
    }

private:
    struct Huffman { short count[16]; short symbol[288]; };
    FILE* f;
    Pipeline& sink;
    unsigned char ibuf[1 << 15];
    size_t ipos = 0, ilen = 0;
    uint32_t bitbuf = 0;
    int bitcnt = 0;
    unsigned char win[32768];
    uint32_t wpos = 0;
    uint64_t total = 0;

    int byte() {
        if (ipos == ilen) {
            ilen = fread(ibuf, 1, sizeof ibuf, f);
            ipos = 0;
            if (ilen == 0) throw std::runtime_error("truncated deflate stream");
        }
        return ibuf[ipos++];
    }
    int bits(int need) {
        uint32_t val = bitbuf;
        while (bitcnt < need) {
            val |= (uint32_t)byte() << bitcnt;
            bitcnt += 8;
        }
        bitbuf = val >> need;
        bitcnt -= need;
        return (int)(val & ((1u << need) - 1));
    }
    void put(unsigned char c) {
        win[wpos++ & 32767] = c;
        ++total;
        sink.put((char)c);
    }
    void stored() {
        bitbuf = 0;
        bitcnt = 0;
        unsigned len = (unsigned)byte();
        len |= (unsigned)byte() << 8;
        unsigned nlen = (unsigned)byte();
        nlen |= (unsigned)byte() << 8;
        if (len != (~nlen & 0xffffu)) throw std::runtime_error("bad stored block");
        while (len--) put((unsigned char)byte());
    }
    static int construct(Huffman& h, const short* length, int n) {
        short offs[16];
        for (int len = 0; len < 16; ++len) h.count[len] = 0;
        for (int s = 0; s < n; ++s) h.count[length[s]]++;
        if (h.count[0] == n) return 0;
        int left = 1;
        for (int len = 1; len < 16; ++len) {
            left <<= 1;
            left -= h.count[len];
            if (left < 0) return left;
        }
        offs[1] = 0;
        for (int len = 1; len < 15; ++len) offs[len + 1] = (short)(offs[len] + h.count[len]);
        for (int s = 0; s < n; ++s)
            if (length[s] != 0) h.symbol[offs[length[s]]++] = (short)s;
        return left;
    }
    int decode(const Huffman& h) {
        int code = 0, first = 0, index = 0;
        for (int len = 1; len < 16; ++len) {
            code |= bits(1);
            int count = h.count[len];
            if (code - count < first) return h.symbol[index + (code - first)];
            index += count;
            first += count;
            first <<= 1;
            code <<= 1;
        }
        return -10;
    }
    void codes(const Huffman& lencode, const Huffman& distcode) {
        static const short lbase[29] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31,
                                        35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258};
        static const short lext[29] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2,
                                       3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0};
        static const short dbase[30] = {1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129,
                                        193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097,
                                        6145, 8193, 12289, 16385, 24577};
        static const short dext[30] = {0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6,
                                       7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13};
        int sym;
        do {
            sym = decode(lencode);
            if (sym < 0) throw std::runtime_error("bad literal/length code");
            if (sym < 256) {
                put((unsigned char)sym);
            } else if (sym > 256) {
                sym -= 257;
                if (sym >= 29) throw std::runtime_error("bad length symbol");
                int len = lbase[sym] + bits(lext[sym]);
                int ds = decode(distcode);
                if (ds < 0 || ds >= 30) throw std::runtime_error("bad distance symbol");
                unsigned dist = (unsigned)(dbase[ds] + bits(dext[ds]));
                if (dist > total) throw std::runtime_error("distance too far back");
                while (len--) put(win[(wpos - dist) & 32767]);
            }
        } while (sym != 256);
    }
    void fixed() {
        static bool built = false;
        static Huffman lencode, distcode;
        if (!built) {
            short lengths[288];
            int s = 0;
            for (; s < 144; ++s) lengths[s] = 8;
            for (; s < 256; ++s) lengths[s] = 9;
            for (; s < 280; ++s) lengths[s] = 7;
            for (; s < 288; ++s) lengths[s] = 8;
            construct(lencode, lengths, 288);
            for (s = 0; s < 30; ++s) lengths[s] = 5;
            construct(distcode, lengths, 30);
            built = true;
        }
        codes(lencode, distcode);
    }
    void dynamic() {
        static const short order[19] = {16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15};
        short lengths[320];
        Huffman lencode, distcode;
        int nlen = bits(5) + 257;
        int ndist = bits(5) + 1;
        int ncode = bits(4) + 4;
        if (nlen > 286 || ndist > 30) throw std::runtime_error("bad dynamic block counts");
        int index;
        for (index = 0; index < ncode; ++index) lengths[order[index]] = (short)bits(3);
        for (; index < 19; ++index) lengths[order[index]] = 0;
        if (construct(lencode, lengths, 19) != 0) throw std::runtime_error("incomplete code-length code");
        index = 0;
        while (index < nlen + ndist) {
            int sym = decode(lencode);
            if (sym < 0) throw std::runtime_error("bad code-length symbol");
            if (sym < 16) {
                lengths[index++] = (short)sym;
            } else {
                short len = 0;
                if (sym == 16) {
                    if (index == 0) throw std::runtime_error("repeat with no previous length");
                    len = lengths[index - 1];
                    sym = 3 + bits(2);
                } else if (sym == 17) {
                    sym = 3 + bits(3);
                } else {
                    sym = 11 + bits(7);
                }
                if (index + sym > nlen + ndist) throw std::runtime_error("too many lengths");
                while (sym--) lengths[index++] = len;
            }
        }
        if (lengths[256] == 0) throw std::runtime_error("no end-of-block code");
        int err = construct(lencode, lengths, nlen);
        if (err && (err < 0 || nlen != lencode.count[0] + lencode.count[1]))
            throw std::runtime_error("bad literal/length lengths");
        err = construct(distcode, lengths + nlen, ndist);
        if (err && (err < 0 || ndist != distcode.count[0] + distcode.count[1]))
            throw std::runtime_error("bad distance lengths");
        codes(lencode, distcode);
    }
};

// ---------------------------------------------------------------------------
// Input: csv file, directory, or zip archive
// ---------------------------------------------------------------------------
static uint32_t rd32(const unsigned char* p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }
static uint16_t rd16(const unsigned char* p) { return (uint16_t)(p[0] | (p[1] << 8)); }

static bool ends_with(const std::string& s, const char* suf) {
    size_t n = strlen(suf);
    return s.size() >= n && s.compare(s.size() - n, n, suf) == 0;
}

// Feed the IMU member of a zip archive into the pipeline. Returns false if not a zip.
static bool feed_zip(FILE* f, Pipeline& pl) {
    if (fseek(f, 0, SEEK_END) != 0) return false;
    long size = ftell(f);
    if (size < 22) return false;
    long tail = std::min<long>(size, 22 + 65535);
    std::vector<unsigned char> buf((size_t)tail);
    fseek(f, size - tail, SEEK_SET);
    if (fread(buf.data(), 1, buf.size(), f) != buf.size()) return false;
    long eocd = -1;
    for (long i = tail - 22; i >= 0; --i)
        if (rd32(&buf[(size_t)i]) == 0x06054b50) { eocd = i; break; }
    if (eocd < 0) return false;
    uint16_t entries = rd16(&buf[(size_t)eocd + 10]);
    uint32_t cd_size = rd32(&buf[(size_t)eocd + 12]);
    uint32_t cd_off = rd32(&buf[(size_t)eocd + 16]);
    if (cd_off == 0xffffffffu) throw std::runtime_error("ZIP64 archives are not supported; unzip first");
    std::vector<unsigned char> cd(cd_size);
    fseek(f, (long)cd_off, SEEK_SET);
    if (fread(cd.data(), 1, cd.size(), f) != cd.size()) throw std::runtime_error("bad zip central directory");
    long best = -1, fallback = -1;
    uint16_t best_method = 0, fb_method = 0;
    uint32_t best_csize = 0, fb_csize = 0;
    size_t p = 0;
    for (int e = 0; e < entries && p + 46 <= cd.size(); ++e) {
        if (rd32(&cd[p]) != 0x02014b50) throw std::runtime_error("bad zip central header");
        uint16_t method = rd16(&cd[p + 10]);
        uint32_t csize = rd32(&cd[p + 20]);
        uint16_t nl = rd16(&cd[p + 28]), xl = rd16(&cd[p + 30]), cl = rd16(&cd[p + 32]);
        uint32_t loff = rd32(&cd[p + 42]);
        std::string name((const char*)&cd[p + 46], nl);
        if (best < 0 && ends_with(name, "imu0/data.csv")) { best = (long)loff; best_method = method; best_csize = csize; }
        if (fallback < 0 && ends_with(name, ".csv")) { fallback = (long)loff; fb_method = method; fb_csize = csize; }
        p += 46 + nl + xl + cl;
    }
    if (best < 0) { best = fallback; best_method = fb_method; best_csize = fb_csize; }
    if (best < 0) throw std::runtime_error("no imu0/data.csv inside the zip");
    unsigned char lh[30];
    fseek(f, best, SEEK_SET);
    if (fread(lh, 1, 30, f) != 30 || rd32(lh) != 0x04034b50) throw std::runtime_error("bad zip local header");
    fseek(f, best + 30 + rd16(lh + 26) + rd16(lh + 28), SEEK_SET);
    if (best_method == 8) {
        Inflater inf(f, pl);
        inf.run();
    } else if (best_method == 0) {
        char b[1 << 15];
        uint32_t left = best_csize;
        while (left > 0) {
            size_t n = fread(b, 1, std::min<size_t>(sizeof b, left), f);
            if (n == 0) throw std::runtime_error("truncated zip member");
            pl.put(b, n);
            left -= (uint32_t)n;
        }
    } else {
        throw std::runtime_error("unsupported zip compression method (use stored or deflate)");
    }
    return true;
}

static fs::path find_csv(const fs::path& dir) {
    fs::path cand;
    std::error_code ec;
    for (auto it = fs::recursive_directory_iterator(dir, ec); it != fs::recursive_directory_iterator(); it.increment(ec)) {
        if (ec) break;
        if (!it->is_regular_file(ec) || it->path().filename() != "data.csv") continue;
        if (it->path().parent_path().filename() == "imu0") return it->path();
        if (cand.empty()) cand = it->path();
    }
    if (cand.empty()) throw std::runtime_error("no imu0/data.csv under " + dir.string());
    return cand;
}

static void feed_input(const std::string& inp, Pipeline& pl) {
    fs::path path(inp);
    if (fs::is_directory(path)) path = find_csv(path);
    FILE* f = fopen(path.string().c_str(), "rb");
    if (!f) throw std::runtime_error("cannot open " + path.string());
    if (!feed_zip(f, pl)) {
        fseek(f, 0, SEEK_SET);
        char b[1 << 15];
        size_t n;
        while ((n = fread(b, 1, sizeof b, f)) > 0) pl.put(b, n);
    }
    fclose(f);
}

static unsigned long long peak_rss_bytes() {
#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS c;
    if (GetProcessMemoryInfo(GetCurrentProcess(), &c, sizeof c)) return c.PeakWorkingSetSize;
    return 0;
#else
    struct rusage r;
    getrusage(RUSAGE_SELF, &r);
#ifdef __APPLE__
    return (unsigned long long)r.ru_maxrss;
#else
    return (unsigned long long)r.ru_maxrss * 1024ULL;
#endif
#endif
}

static std::string json_escape(const std::string& s) {
    std::string o;
    for (char c : s) {
        if (c == '\\' || c == '"') { o += '\\'; o += c; }
        else if ((unsigned char)c < 0x20) { char b[8]; snprintf(b, sizeof b, "\\u%04x", c); o += b; }
        else o += c;
    }
    return o;
}

static void usage() {
    fprintf(stderr,
            "usage: pathfinder <imu0/data.csv | dataset_dir | dataset.zip> [-o estimated_trajectory.txt]\n"
            "                  [--metadata metadata.json] [--no-extrapolate] [--no-smooth] [--set key=val ...]\n");
}

int main(int argc, char** argv) {
    std::string input, output = "estimated_trajectory.txt", metadata;
    bool extrapolate = true, no_smooth = false;
    Params prm;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if ((a == "-o" || a == "--output") && i + 1 < argc) {
            output = argv[++i];
        } else if (a == "--metadata" && i + 1 < argc) {
            metadata = argv[++i];
        } else if (a == "--no-extrapolate") {
            extrapolate = false;
        } else if (a == "--no-smooth") {
            no_smooth = true;
        } else if (a == "--set") {
            while (i + 1 < argc && argv[i + 1][0] != '-' && strchr(argv[i + 1], '=')) {
                std::string kv = argv[++i];
                size_t eq = kv.find('=');
                std::string k = kv.substr(0, eq);
                bool found = false;
                for (auto& e : prm.table())
                    if (k == e.name) { *e.v = atof(kv.c_str() + eq + 1); found = true; }
                if (!found) { fprintf(stderr, "unknown parameter %s\n", k.c_str()); return 2; }
            }
        } else if (a == "-h" || a == "--help") {
            usage();
            return 0;
        } else if (input.empty() && (a.empty() || a[0] != '-')) {
            input = a;
        } else {
            usage();
            return 2;
        }
    }
    if (input.empty()) { usage(); return 2; }
    if (no_smooth) prm.smooth = 0.0;

    auto t0 = std::chrono::steady_clock::now();
    FILE* out = fopen(output.c_str(), "wb");
    if (!out) { fprintf(stderr, "cannot write %s\n", output.c_str()); return 1; }
    fputs("# timestamp tx ty tz qx qy qz qw\n", out);
    Pipeline pl(prm, out, extrapolate);
    try {
        feed_input(input, pl);
        pl.finish();
    } catch (const std::exception& e) {
        fclose(out);
        fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    fclose(out);
    double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    double span = pl.has_last ? (double)(pl.last_ts - pl.t_first) * 1e-9 : 0.0;
    double ms_per_s = span > 0 ? 1000.0 * elapsed / span : 0.0;
    unsigned long long rss = peak_rss_bytes();
    fprintf(stderr, "[pathfinder] %lld poses (+%lld extrapolated) -> %s | %.3f ms per IMU-second | "
                    "peak RSS %.2f MB | invalid=%lld gaps=%lld spikes=%lld\n",
            pl.n_out, pl.n_extra, output.c_str(), ms_per_s, rss / 1048576.0,
            pl.est.n_invalid, pl.est.n_gaps, pl.est.n_spikes);
    if (!metadata.empty()) {
        FILE* m = fopen(metadata.c_str(), "wb");
        if (m) {
            std::error_code ec;
            fprintf(m, "{\n");
            fprintf(m, "  \"implementation\": \"C++17 (src/main.cpp)\",\n");
            fprintf(m, "  \"input\": \"%s\",\n", json_escape(fs::absolute(input, ec).string()).c_str());
            fprintf(m, "  \"output\": \"%s\",\n", json_escape(fs::absolute(output, ec).string()).c_str());
            fprintf(m, "  \"imu_rows\": %lld,\n", pl.n_out);
            fprintf(m, "  \"extrapolated_rows\": %lld,\n", pl.n_extra);
            fprintf(m, "  \"nominal_rate_hz\": %.3f,\n", 1e9 / (double)pl.dt_nom_ns);
            fprintf(m, "  \"imu_span_s\": %.3f,\n", span);
            fprintf(m, "  \"static_initialisation\": %s,\n", pl.est.init_static ? "true" : "false");
            fprintf(m, "  \"invalid_rows\": %lld,\n", pl.est.n_invalid);
            fprintf(m, "  \"timestamp_gaps\": %lld,\n", pl.est.n_gaps);
            fprintf(m, "  \"spikes_rejected\": %lld,\n", pl.est.n_spikes);
            fprintf(m, "  \"blind_time_s\": %.3f,\n", pl.est.blind_time);
            fprintf(m, "  \"drag_active_s\": %.3f,\n", pl.est.drag_gate_time);
            fprintf(m, "  \"drag_final_gate\": %.3f,\n", pl.est.dgate);
            fprintf(m, "  \"smoothing\": \"%s\",\n", prm.smooth > 0.0 ? "two-pass zero-phase" : "causal (single pass)");
            fprintf(m, "  \"runtime_s\": %.4f,\n", elapsed);
            fprintf(m, "  \"runtime_ms_per_imu_second\": %.4f,\n", ms_per_s);
            fprintf(m, "  \"peak_rss_bytes\": %llu,\n", rss);
            fprintf(m, "  \"peak_rss_MB\": %.3f,\n", rss / 1048576.0);
            fprintf(m, "  \"threads\": 1,\n");
            fprintf(m, "  \"parameters\": {");
            bool first = true;
            for (auto& e : prm.table()) {
                fprintf(m, "%s\n    \"%s\": %.17g", first ? "" : ",", e.name, *e.v);
                first = false;
            }
            fprintf(m, "\n  }\n}\n");
            fclose(m);
        }
    }
    return 0;
}
