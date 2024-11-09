//  Copyright (c) 2024 Feng Yang
//
//  I am making my contributions/submissions to this project solely in my
//  personal capacity and am not conveying any rights to any intellectual
//  property of any third parties.

#include <nanobind/nanobind.h>

namespace nb = nanobind;
using namespace nb::literals;

void bind_bbox3d(nb::module_& m);

NB_MODULE(py_usd_ext, m) {
    m.doc() = "python binding for usd";

    auto base = m.def_submodule("base");
    auto gf = base.def_submodule("gf");
    bind_bbox3d(gf);
}
