//
// Copyright 2021 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
////////////////////////////////////////////////////////////////////////
// This file is generated by a script.  Do not edit directly.  Edit the
// dualQuat.template.cpp file to make changes.

#include "pxr/pxr.h"
#include "pxr/base/gf/dualQuatd.h"
#include "pxr/base/gf/ostreamHelpers.h"
#include "pxr/base/tf/type.h"

#include "pxr/base/gf/dualQuatf.h"
#include "pxr/base/gf/dualQuath.h"

PXR_NAMESPACE_OPEN_SCOPE

TF_REGISTRY_FUNCTION(TfType) {
    TfType::Define<GfDualQuatd>();
}

GfDualQuatd::GfDualQuatd(const GfDualQuatf& other) : _real(other.GetReal()), _dual(other.GetDual()) {}
GfDualQuatd::GfDualQuatd(const GfDualQuath& other) : _real(other.GetReal()), _dual(other.GetDual()) {}

std::pair<double, double> GfDualQuatd::GetLength() const {
    const double realLength = _real.GetLength();

    if (realLength == 0) return std::pair<double, double>{0, 0};

    return std::pair<double, double>{realLength, GfDot(_real, _dual) / realLength};
}

GfDualQuatd GfDualQuatd::GetNormalized(double eps) const {
    GfDualQuatd dq(*this);
    dq.Normalize(eps);

    return dq;
}

std::pair<double, double> GfDualQuatd::Normalize(double eps) {
    const std::pair<double, double> length = GetLength();
    const double realLength = length.first;

    if (realLength < eps) {
        (*this) = GfDualQuatd::GetIdentity();
    } else {
        const double invRealLength = 1.0 / realLength;
        _real *= invRealLength;
        _dual *= invRealLength;

        _dual -= (GfDot(_real, _dual) * _real);
    }

    return length;
}

GfDualQuatd GfDualQuatd::GetConjugate() const {
    return GfDualQuatd(_real.GetConjugate(), _dual.GetConjugate());
}

GfDualQuatd GfDualQuatd::GetInverse() const {
    // DQ * DQ.GetInverse() == GetIdentity()
    const double realLengthSqr = GfDot(_real, _real);

    if (realLengthSqr <= 0.0) return GfDualQuatd::GetIdentity();

    const double invRealLengthSqr = 1.0 / realLengthSqr;
    const GfDualQuatd conjInvLenSqr = GetConjugate() * invRealLengthSqr;
    const GfQuatd realPart = conjInvLenSqr.GetReal();
    const GfQuatd dualPart =
            conjInvLenSqr.GetDual() - (2.0 * invRealLengthSqr * GfDot(_real, _dual) * conjInvLenSqr.GetReal());

    return GfDualQuatd(realPart, dualPart);
}

void GfDualQuatd::SetTranslation(const GfVec3d& translation) {
    // compute and set the dual part
    _dual = GfQuatd(0.0, 0.5 * translation) * _real;
}

GfVec3d GfDualQuatd::GetTranslation() const {
    // _dual = GfQuatd(0, 0.5*translation) * _real
    // => translation = 2 * (_dual * _real.GetConjugate()).GetImaginary()

    // Assume that this dual quaternion is normalized
    TF_DEV_AXIOM(GfIsClose(_real.GetLength(), 1.0, 0.001));
    const double r1 = _dual.GetReal();
    const double r2 = _real.GetReal();
    const GfVec3d& i1 = _dual.GetImaginary();
    const GfVec3d& i2 = _real.GetImaginary();

    // Translation of the dual quaternion: -2.0 * (r1*i2 - r2*i1 + i1^i2)
    return GfVec3d(-2.0 * (r1 * i2[0] - r2 * i1[0] + (i1[1] * i2[2] - i1[2] * i2[1])),
                   -2.0 * (r1 * i2[1] - r2 * i1[1] + (i1[2] * i2[0] - i1[0] * i2[2])),
                   -2.0 * (r1 * i2[2] - r2 * i1[2] + (i1[0] * i2[1] - i1[1] * i2[0])));
}

GfDualQuatd& GfDualQuatd::operator*=(const GfDualQuatd& dq) {
    const GfQuatd tempReal = GetReal() * dq.GetReal();
    const GfQuatd tempDual = GetReal() * dq.GetDual() + GetDual() * dq.GetReal();

    SetReal(tempReal);
    SetDual(tempDual);

    return *this;
}

GfVec3d GfDualQuatd::Transform(const GfVec3d& vec) const {
    // Apply rotation and translation
    return GetReal().Transform(vec) + GetTranslation();
}

std::ostream& operator<<(std::ostream& out, const GfDualQuatd& dq) {
    return (out << '(' << Gf_OstreamHelperP(dq.GetReal()) << ", " << Gf_OstreamHelperP(dq.GetDual()) << ')');
}

PXR_NAMESPACE_CLOSE_SCOPE
