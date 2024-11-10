//
// Copyright 2017 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#ifndef PXR_BASE_WORK_API_H
#define PXR_BASE_WORK_API_H

#include "pxr/base/arch/export.h"

#if defined(PXR_STATIC)
#define WORK_API
#define WORK_API_TEMPLATE_CLASS(...)
#define WORK_API_TEMPLATE_STRUCT(...)
#define WORK_LOCAL
#else
#if defined(WORK_EXPORTS)
#define WORK_API ARCH_EXPORT
#define WORK_API_TEMPLATE_CLASS(...) ARCH_EXPORT_TEMPLATE(class, __VA_ARGS__)
#define WORK_API_TEMPLATE_STRUCT(...) ARCH_EXPORT_TEMPLATE(struct, __VA_ARGS__)
#else
#define WORK_API ARCH_IMPORT
#define WORK_API_TEMPLATE_CLASS(...) ARCH_IMPORT_TEMPLATE(class, __VA_ARGS__)
#define WORK_API_TEMPLATE_STRUCT(...) ARCH_IMPORT_TEMPLATE(struct, __VA_ARGS__)
#endif
#define WORK_LOCAL ARCH_HIDDEN
#endif

#endif
