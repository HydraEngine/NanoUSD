//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include "pxr/usd/usdSkel/animation.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType) {
    TfType::Define<UsdSkelAnimation, TfType::Bases<UsdTyped>>();

    // Register the usd prim typename as an alias under UsdSchemaBase. This
    // enables one to call
    // TfType::Find<UsdSchemaBase>().FindDerivedByName("SkelAnimation")
    // to find TfType<UsdSkelAnimation>, which is how IsA queries are
    // answered.
    TfType::AddAlias<UsdSchemaBase, UsdSkelAnimation>("SkelAnimation");
}

/* virtual */
UsdSkelAnimation::~UsdSkelAnimation() {}

/* static */
UsdSkelAnimation UsdSkelAnimation::Get(const UsdStagePtr& stage, const SdfPath& path) {
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return UsdSkelAnimation();
    }
    return UsdSkelAnimation(stage->GetPrimAtPath(path));
}

/* static */
UsdSkelAnimation UsdSkelAnimation::Define(const UsdStagePtr& stage, const SdfPath& path) {
    static TfToken usdPrimTypeName("SkelAnimation");
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return UsdSkelAnimation();
    }
    return UsdSkelAnimation(stage->DefinePrim(path, usdPrimTypeName));
}

/* virtual */
UsdSchemaKind UsdSkelAnimation::_GetSchemaKind() const {
    return UsdSkelAnimation::schemaKind;
}

/* static */
const TfType& UsdSkelAnimation::_GetStaticTfType() {
    static TfType tfType = TfType::Find<UsdSkelAnimation>();
    return tfType;
}

/* static */
bool UsdSkelAnimation::_IsTypedSchema() {
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType& UsdSkelAnimation::_GetTfType() const {
    return _GetStaticTfType();
}

UsdAttribute UsdSkelAnimation::GetJointsAttr() const {
    return GetPrim().GetAttribute(UsdSkelTokens->joints);
}

UsdAttribute UsdSkelAnimation::CreateJointsAttr(VtValue const& defaultValue, bool writeSparsely) const {
    return UsdSchemaBase::_CreateAttr(UsdSkelTokens->joints, SdfValueTypeNames->TokenArray,
                                      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute UsdSkelAnimation::GetTranslationsAttr() const {
    return GetPrim().GetAttribute(UsdSkelTokens->translations);
}

UsdAttribute UsdSkelAnimation::CreateTranslationsAttr(VtValue const& defaultValue, bool writeSparsely) const {
    return UsdSchemaBase::_CreateAttr(UsdSkelTokens->translations, SdfValueTypeNames->Float3Array,
                                      /* custom = */ false, SdfVariabilityVarying, defaultValue, writeSparsely);
}

UsdAttribute UsdSkelAnimation::GetRotationsAttr() const {
    return GetPrim().GetAttribute(UsdSkelTokens->rotations);
}

UsdAttribute UsdSkelAnimation::CreateRotationsAttr(VtValue const& defaultValue, bool writeSparsely) const {
    return UsdSchemaBase::_CreateAttr(UsdSkelTokens->rotations, SdfValueTypeNames->QuatfArray,
                                      /* custom = */ false, SdfVariabilityVarying, defaultValue, writeSparsely);
}

UsdAttribute UsdSkelAnimation::GetScalesAttr() const {
    return GetPrim().GetAttribute(UsdSkelTokens->scales);
}

UsdAttribute UsdSkelAnimation::CreateScalesAttr(VtValue const& defaultValue, bool writeSparsely) const {
    return UsdSchemaBase::_CreateAttr(UsdSkelTokens->scales, SdfValueTypeNames->Half3Array,
                                      /* custom = */ false, SdfVariabilityVarying, defaultValue, writeSparsely);
}

UsdAttribute UsdSkelAnimation::GetBlendShapesAttr() const {
    return GetPrim().GetAttribute(UsdSkelTokens->blendShapes);
}

UsdAttribute UsdSkelAnimation::CreateBlendShapesAttr(VtValue const& defaultValue, bool writeSparsely) const {
    return UsdSchemaBase::_CreateAttr(UsdSkelTokens->blendShapes, SdfValueTypeNames->TokenArray,
                                      /* custom = */ false, SdfVariabilityUniform, defaultValue, writeSparsely);
}

UsdAttribute UsdSkelAnimation::GetBlendShapeWeightsAttr() const {
    return GetPrim().GetAttribute(UsdSkelTokens->blendShapeWeights);
}

UsdAttribute UsdSkelAnimation::CreateBlendShapeWeightsAttr(VtValue const& defaultValue, bool writeSparsely) const {
    return UsdSchemaBase::_CreateAttr(UsdSkelTokens->blendShapeWeights, SdfValueTypeNames->FloatArray,
                                      /* custom = */ false, SdfVariabilityVarying, defaultValue, writeSparsely);
}

namespace {
static inline TfTokenVector _ConcatenateAttributeNames(const TfTokenVector& left, const TfTokenVector& right) {
    TfTokenVector result;
    result.reserve(left.size() + right.size());
    result.insert(result.end(), left.begin(), left.end());
    result.insert(result.end(), right.begin(), right.end());
    return result;
}
}  // namespace

/*static*/
const TfTokenVector& UsdSkelAnimation::GetSchemaAttributeNames(bool includeInherited) {
    static TfTokenVector localNames = {
            UsdSkelTokens->joints, UsdSkelTokens->translations, UsdSkelTokens->rotations,
            UsdSkelTokens->scales, UsdSkelTokens->blendShapes,  UsdSkelTokens->blendShapeWeights,
    };
    static TfTokenVector allNames = _ConcatenateAttributeNames(UsdTyped::GetSchemaAttributeNames(true), localNames);

    if (includeInherited)
        return allNames;
    else
        return localNames;
}

PXR_NAMESPACE_CLOSE_SCOPE

// ===================================================================== //
// Feel free to add custom code below this line. It will be preserved by
// the code generator.
//
// Just remember to wrap code in the appropriate delimiters:
// 'PXR_NAMESPACE_OPEN_SCOPE', 'PXR_NAMESPACE_CLOSE_SCOPE'.
// ===================================================================== //
// --(BEGIN CUSTOM CODE)--

#include "pxr/usd/usdSkel/utils.h"

PXR_NAMESPACE_OPEN_SCOPE

bool UsdSkelAnimation::GetTransforms(VtMatrix4dArray* xforms, UsdTimeCode time) const {
    VtVec3fArray translations;
    if (GetTranslationsAttr().Get(&translations, time)) {
        VtQuatfArray rotations;
        if (GetRotationsAttr().Get(&rotations, time)) {
            VtVec3hArray scales;
            if (GetScalesAttr().Get(&scales, time)) {
                return UsdSkelMakeTransforms(translations, rotations, scales, xforms);
            }
        }
    }
    return false;
}

bool UsdSkelAnimation::SetTransforms(const VtMatrix4dArray& xforms, UsdTimeCode time) const {
    VtVec3fArray translations;
    VtQuatfArray rotations;
    VtVec3hArray scales;
    if (UsdSkelDecomposeTransforms(xforms, &translations, &rotations, &scales)) {
        return GetTranslationsAttr().Set(translations, time) & GetRotationsAttr().Set(rotations, time) &
               GetScalesAttr().Set(scales, time);
    }
    return false;
}

PXR_NAMESPACE_CLOSE_SCOPE
