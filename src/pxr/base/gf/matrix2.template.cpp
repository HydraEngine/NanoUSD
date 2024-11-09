//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
////////////////////////////////////////////////////////////////////////
// This file is generated by a script.  Do not edit directly.  Edit the
// matrix2.template.cpp file to make changes.

{% extends "matrix.template.cpp" %}

{% block customFunctions %}
{{ MAT }}
{{ MAT }}::GetInverse(double *detPtr, double eps) const
{
    double det = GetDeterminant();

    if (detPtr) {
        // CODE_COVERAGE_OFF_NO_REPORT This is inaccessible from script and not
        // worth writing a whole C++ test for.
	*detPtr = det;
        // CODE_COVERAGE_ON_NO_REPORT
    }

    {{ MAT }} inverse;

    if (GfAbs(det) > eps) {

        double rcp = 1.0 / det;

{%- macro SCALAR_CAST(t) %}
{%- if SCL == 'float' %}
static_cast<float>({{ t }})
{%- else %}
{{ t }}
{%- endif %}
{% endmacro %}

        inverse._mtx[0][0] = {{ SCALAR_CAST("_mtx[1][1]*rcp") }};
        inverse._mtx[0][1] = {{ SCALAR_CAST("_mtx[0][1]*-rcp") }};
        inverse._mtx[1][0] = {{ SCALAR_CAST("_mtx[1][0]*-rcp") }};
        inverse._mtx[1][1] = {{ SCALAR_CAST("_mtx[0][0]*rcp") }};
    }
    else {
	inverse.SetDiagonal(FLT_MAX);
    }

    return inverse;

}

double
{{ MAT }}::GetDeterminant() const
{
    return (_mtx[0][0] * _mtx[1][1] - _mtx[0][1] * _mtx[1][0]);
}
{% endblock customFunctions %}
