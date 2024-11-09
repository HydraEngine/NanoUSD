//  Copyright (c) 2024 Feng Yang
//
//  I am making my contributions/submissions to this project solely in my
//  personal capacity and am not conveying any rights to any intellectual
//  property of any third parties.

#include <pxr/base/gf/bbox3d.h>

#include <nanobind/nanobind.h>

namespace nb = nanobind;
using namespace nb::literals;
using namespace pxr;

void bind_bbox3d(nb::module_& m) {
    nb::class_<GfBBox3d>(m, "GfBBox3d")
            .def(nb::init<>())
            .def("Set", &GfBBox3d::Set)
            .def("SetMatrix", &GfBBox3d::SetMatrix)
            .def("SetRange", &GfBBox3d::SetRange)
            .def("GetRange", &GfBBox3d::GetRange)
            .def("GetBox", &GfBBox3d::GetBox);
}
