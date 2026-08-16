#pragma once

// Расчёт ориентации: углы Эйлера–Крылова и их производные.

#include <cmath>

#include "../math_lib/interpolation.h"
#include "../math_lib/matrix_ops.h"
#include "../math_lib/transformations.h"
#include "../utils/constants.h"
#include "../utils/types.h"

namespace ins
{

// углы ориентации, рад
struct Attitude
{
    double heading = 0; // ψ — рыскание (против ЧС от Севера)
    double pitch = 0;   // θ — тангаж
    double roll = 0;    // γ — крен
};

// производные углов ориентации, рад/с
struct AttitudeRates
{
    double heading_dot = 0;
    double pitch_dot = 0;
    double roll_dot = 0;
};

// относительная угловая скорость: ω_отн = ω_абс − Cᵀ·ω_геогр
inline Vector relativeRate(const Vector &w_abs, const Vector &w_nav, const Matrix &C)
{
    return vector_diff(w_abs, navToBody(C, w_nav));
}

// кинематические уравнения Эйлера–Крылова (3.30)
inline AttitudeRates eulerRates(const Vector &w_rel, double pitch, double roll)
{
    const double wx = w_rel[0];
    const double wy = w_rel[1];
    const double wz = w_rel[2];

    AttitudeRates rates;
    rates.heading_dot = (wy * cos(roll) - wz * sin(roll)) / cos(pitch);
    rates.pitch_dot = wy * sin(roll) + wz * cos(roll);
    rates.roll_dot = wx - tan(pitch) * (wy * cos(roll) - wz * sin(roll));
    return rates;
}

// интегрирование углов методом трапеций с нормированием
inline Attitude integrate(const Attitude &prev, const AttitudeRates &now,
                          const AttitudeRates &prev_rates, double dt)
{
    Attitude att;
    att.heading = normolize_angle(v_integral(now.heading_dot, prev.heading, prev_rates.heading_dot, dt));
    att.pitch = normolize_angle(v_integral(now.pitch_dot, prev.pitch, prev_rates.pitch_dot, dt));
    att.roll = normolize_angle(v_integral(now.roll_dot, prev.roll, prev_rates.roll_dot, dt));
    return att;
}

// Оценка курса по горизонтальной проекции угловой скорости Земли
// (гирокомпасирование). В связанной СК (X вперёд, Z вправо) для ровного полёта:
//   wx ≈ U·cosφ·cosψ,  wz ≈ −U·cosφ·sinψ  ⇒  ψ = atan2(−wz, wx).
// atan2 даёт полный квадрант и в отличие от acos не выдаёт NaN и не
// «зеркалит» отрицательные курсы. Смещения гироскопов должны быть уже
// вычтены (иначе сигнал ω Земли тонет в смещении ~1e-3 рад/с).
inline double headingFromEarthRate(double wx, double wz)
{
    return normolize_angle(atan2(-wz, wx));
}

} // namespace ins
