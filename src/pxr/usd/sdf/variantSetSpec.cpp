//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
/// \file VariantSetSpec.cpp

#include "pxr/pxr.h"
#include "pxr/usd/sdf/variantSetSpec.h"
#include "pxr/usd/sdf/childrenUtils.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/sdf/primSpec.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/variantSpec.h"
#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/type.h"
#include "pxr/base/trace/trace.h"
#include <ostream>

PXR_NAMESPACE_OPEN_SCOPE

SDF_DEFINE_SPEC(SdfSchema, SdfSpecTypeVariantSet, SdfVariantSetSpec, SdfSpec);

SdfVariantSetSpecHandle SdfVariantSetSpec::New(const SdfPrimSpecHandle& owner, const std::string& name) {
    TRACE_FUNCTION();

    if (!owner) {
        TF_CODING_ERROR("NULL owner prim");
        return TfNullPtr;
    }

    if (!Sdf_ChildrenUtils<Sdf_VariantSetChildPolicy>::IsValidName(name)) {
        TF_CODING_ERROR(
                "Cannot create variant set spec with invalid "
                "identifier: '%s'",
                name.c_str());
        return TfNullPtr;
    }

    SdfChangeBlock block;

    SdfLayerHandle layer = owner->GetLayer();
    SdfPath path = owner->GetPath().AppendVariantSelection(name, "");

    if (!path.IsPrimVariantSelectionPath()) {
        TF_CODING_ERROR(
                "Cannot create variant set spec at invalid "
                "path <%s{%s=}>",
                owner->GetPath().GetText(), name.c_str());
        return TfNullPtr;
    }

    if (!Sdf_ChildrenUtils<Sdf_VariantSetChildPolicy>::CreateSpec(layer, path, SdfSpecTypeVariantSet)) return TfNullPtr;

    return TfStatic_cast<SdfVariantSetSpecHandle>(layer->GetObjectAtPath(path));
}

SdfVariantSetSpecHandle SdfVariantSetSpec::New(const SdfVariantSpecHandle& owner, const std::string& name) {
    TRACE_FUNCTION();

    if (!owner) {
        TF_CODING_ERROR("NULL owner variant");
        return TfNullPtr;
    }

    if (!Sdf_ChildrenUtils<Sdf_VariantSetChildPolicy>::IsValidName(name)) {
        TF_CODING_ERROR(
                "Cannot create variant set spec with invalid "
                "identifier: '%s'",
                name.c_str());
        return TfNullPtr;
    }

    SdfChangeBlock block;

    SdfLayerHandle layer = owner->GetLayer();
    SdfPath path = owner->GetPath().AppendVariantSelection(name, "");

    if (!path.IsPrimVariantSelectionPath()) {
        TF_CODING_ERROR(
                "Cannot create variant set spec at invalid "
                "path <%s{%s=}>",
                owner->GetPath().GetText(), name.c_str());
        return TfNullPtr;
    }

    if (!Sdf_ChildrenUtils<Sdf_VariantSetChildPolicy>::CreateSpec(layer, path, SdfSpecTypeVariantSet)) return TfNullPtr;

    return TfStatic_cast<SdfVariantSetSpecHandle>(layer->GetObjectAtPath(path));
}

//
// Name
//

std::string SdfVariantSetSpec::GetName() const {
    return GetPath().GetName();
}

TfToken SdfVariantSetSpec::GetNameToken() const {
    return GetPath().GetNameToken();
}

//
// Namespace hierarchy
//

SdfSpecHandle SdfVariantSetSpec::GetOwner() const {
    return GetLayer()->GetObjectAtPath(GetPath().GetParentPath());
}

//
// Variants
//

SdfVariantView SdfVariantSetSpec::GetVariants() const {
    return SdfVariantView(GetLayer(), GetPath(), SdfChildrenKeys->VariantChildren);
}

SdfVariantSpecHandleVector SdfVariantSetSpec::GetVariantList() const {
    return GetVariants().values();
}

void SdfVariantSetSpec::RemoveVariant(const SdfVariantSpecHandle& variant) {
    const SdfLayerHandle& layer = GetLayer();
    const SdfPath& path = GetPath();

    SdfPath parentPath = Sdf_VariantChildPolicy::GetParentPath(variant->GetPath());
    if (variant->GetLayer() != layer || parentPath != path) {
        TF_CODING_ERROR(
                "Cannot remove a variant that does not belong to "
                "this variant set.");
        return;
    }

    if (!Sdf_ChildrenUtils<Sdf_VariantChildPolicy>::RemoveChild(layer, path, variant->GetNameToken())) {
        TF_CODING_ERROR("Unable to remove child: %s", variant->GetName().c_str());
    }
}

PXR_NAMESPACE_CLOSE_SCOPE
