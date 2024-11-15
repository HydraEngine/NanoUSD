//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include "pxr/pxr.h"
#include "pxr/usd/usd/common.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/instanceCache.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/valueUtils.h"

#include "pxr/usd/sdf/attributeSpec.h"
#include "pxr/usd/sdf/changeBlock.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/sdf/primSpec.h"
#include "pxr/usd/sdf/relationshipSpec.h"
#include "pxr/usd/sdf/schema.h"
#include "pxr/base/trace/trace.h"

#include <algorithm>
#include <set>
#include <vector>

PXR_NAMESPACE_OPEN_SCOPE

// ------------------------------------------------------------------------- //
// UsdRelationship
// ------------------------------------------------------------------------- //

static SdfPath _MapTargetPath(const UsdStage* stage, const SdfPath& anchor, const SdfPath& target) {
    // If this is a relative target path, we have to map both the anchor
    // and target path and then re-relativize them.
    const UsdEditTarget& editTarget = stage->GetEditTarget();
    if (target.IsAbsolutePath()) {
        return editTarget.MapToSpecPath(target).StripAllVariantSelections();
    } else {
        const SdfPath anchorPrim = anchor.GetPrimPath();
        const SdfPath translatedAnchorPrim = editTarget.MapToSpecPath(anchorPrim).StripAllVariantSelections();
        const SdfPath translatedTarget =
                editTarget.MapToSpecPath(target.MakeAbsolutePath(anchorPrim)).StripAllVariantSelections();
        return translatedTarget.MakeRelativePath(translatedAnchorPrim);
    }
}

SdfPath UsdRelationship::_GetTargetForAuthoring(const SdfPath& target, std::string* whyNot) const {
    if (!target.IsEmpty()) {
        SdfPath absTarget = target.MakeAbsolutePath(GetPath().GetAbsoluteRootOrPrimPath());
        if (Usd_InstanceCache::IsPathInPrototype(absTarget)) {
            if (whyNot) {
                *whyNot =
                        "Cannot target a prototype or an object within a "
                        "prototype.";
            }
            return SdfPath();
        }
    }

    UsdStage* stage = _GetStage();
    SdfPath mappedPath = _MapTargetPath(stage, GetPath(), target);
    if (mappedPath.IsEmpty()) {
        if (whyNot) {
            *whyNot = TfStringPrintf(
                    "Cannot map <%s> to layer @%s@ via stage's "
                    "EditTarget",
                    target.GetText(), stage->GetEditTarget().GetLayer()->GetIdentifier().c_str());
        }
    }

    return mappedPath;
}

bool UsdRelationship::AddTarget(const SdfPath& target, UsdListPosition position) const {
    std::string errMsg;
    const SdfPath targetToAuthor = _GetTargetForAuthoring(target, &errMsg);
    if (targetToAuthor.IsEmpty()) {
        TF_CODING_ERROR("Cannot add target <%s> to relationship <%s>: %s", target.GetText(), GetPath().GetText(),
                        errMsg.c_str());
        return false;
    }

    // NOTE! Do not insert any code that modifies scene description between the
    // changeblock and the call to _CreateSpec!  Explanation: _CreateSpec calls
    // code that inspects the composition graph and then does some authoring.
    // We want that authoring to be inside the change block, but if any scene
    // description changes are made after the block is created but before we
    // call _CreateSpec, the composition structure may be invalidated.
    SdfChangeBlock block;
    SdfRelationshipSpecHandle relSpec = _CreateSpec();

    if (!relSpec) return false;

    Usd_InsertListItem(relSpec->GetTargetPathList(), targetToAuthor, position);
    return true;
}

bool UsdRelationship::RemoveTarget(const SdfPath& target) const {
    std::string errMsg;
    const SdfPath targetToAuthor = _GetTargetForAuthoring(target, &errMsg);
    if (targetToAuthor.IsEmpty()) {
        TF_CODING_ERROR("Cannot remove target <%s> from relationship <%s>: %s", target.GetText(), GetPath().GetText(),
                        errMsg.c_str());
        return false;
    }

    // NOTE! Do not insert any code that modifies scene description between the
    // changeblock and the call to _CreateSpec!  Explanation: _CreateSpec calls
    // code that inspects the composition graph and then does some authoring.
    // We want that authoring to be inside the change block, but if any scene
    // description changes are made after the block is created but before we
    // call _CreateSpec, the composition structure may be invalidated.
    SdfChangeBlock block;
    SdfRelationshipSpecHandle relSpec = _CreateSpec();

    if (!relSpec) return false;

    relSpec->GetTargetPathList().Remove(targetToAuthor);
    return true;
}

bool UsdRelationship::SetTargets(const SdfPathVector& targets) const {
    SdfPathVector mappedPaths;
    mappedPaths.reserve(targets.size());
    for (const SdfPath& target : targets) {
        std::string errMsg;
        mappedPaths.push_back(_GetTargetForAuthoring(target, &errMsg));
        if (mappedPaths.back().IsEmpty()) {
            TF_CODING_ERROR("Cannot set target <%s> on relationship <%s>: %s", target.GetText(), GetPath().GetText(),
                            errMsg.c_str());
            return false;
        }
    }

    // NOTE! Do not insert any code that modifies scene description between the
    // changeblock and the call to _CreateSpec!  Explanation: _CreateSpec calls
    // code that inspects the composition graph and then does some authoring.
    // We want that authoring to be inside the change block, but if any scene
    // description changes are made after the block is created but before we
    // call _CreateSpec, the composition structure may be invalidated.
    SdfChangeBlock block;
    SdfRelationshipSpecHandle relSpec = _CreateSpec();

    if (!relSpec) return false;

    relSpec->GetTargetPathList().ClearEditsAndMakeExplicit();
    relSpec->GetTargetPathList().GetExplicitItems() = mappedPaths;

    return true;
}

bool UsdRelationship::ClearTargets(bool removeSpec) const {
    // NOTE! Do not insert any code that modifies scene description between the
    // changeblock and the call to _CreateSpec!  Explanation: _CreateSpec calls
    // code that inspects the composition graph and then does some authoring.
    // We want that authoring to be inside the change block, but if any scene
    // description changes are made after the block is created but before we
    // call _CreateSpec, the composition structure may be invalidated.
    SdfChangeBlock block;
    SdfRelationshipSpecHandle relSpec = _CreateSpec();

    if (!relSpec) return false;

    if (removeSpec) {
        SdfPrimSpecHandle owner = TfDynamic_cast<SdfPrimSpecHandle>(relSpec->GetOwner());
        owner->RemoveProperty(relSpec);
    } else {
        relSpec->GetTargetPathList().ClearEdits();
    }
    return true;
}

bool UsdRelationship::GetTargets(SdfPathVector* targets) const {
    TRACE_FUNCTION();
    return _GetTargets(SdfSpecTypeRelationship, targets);
}

bool UsdRelationship::_GetForwardedTargetsImpl(SdfPathSet* visited,
                                               SdfPathSet* uniqueTargets,
                                               SdfPathVector* targets,
                                               bool* foundAnyErrors,
                                               bool includeForwardingRels) const {
    SdfPathVector curTargets;
    // Get the targets for this relationship first.
    const bool foundTargets = _GetTargets(SdfSpecTypeRelationship, &curTargets, foundAnyErrors);
    // If there are no targets we can just return the return value of
    // _GetTargets. Note that this may be true if there are explicit opinions
    // that make the targets empty.
    if (curTargets.empty()) {
        return foundTargets;
    }

    // We'll only return success if this relationship provides a target to the
    // list or one of its forwarded relationships returns success (which could
    // be because of a explicit empty opinion)
    bool success = false;

    // Process all targets at this relationship.
    for (SdfPath const& target : curTargets) {
        if (target.IsPrimPropertyPath()) {
            // Resolve forwarding if this target points at a relationship.
            if (UsdPrim prim = GetStage()->GetPrimAtPath(target.GetPrimPath())) {
                if (UsdRelationship rel = prim.GetRelationship(target.GetNameToken())) {
                    // Only do this rel if we've not yet seen it.
                    if (visited->insert(rel.GetPath()).second) {
                        success |= rel._GetForwardedTargetsImpl(visited, uniqueTargets, targets, foundAnyErrors,
                                                                includeForwardingRels);
                    }
                    if (!includeForwardingRels) continue;
                }
            }
        }
        // If we're adding a target then this relationship is providing a
        // forwarded target opinion so we'll return success at the end.
        success = true;
        if (uniqueTargets->insert(target).second) targets->push_back(target);
    }

    // foundAnyErrors is passed through every level of recursion so we don't
    // include it here; it will be accounted for before _GetForwardedTargets
    // returns.
    return success;
}

bool UsdRelationship::_GetForwardedTargets(SdfPathVector* targets, bool includeForwardingRels) const {
    SdfPathSet visited, uniqueTargets;
    bool foundAnyErrors = false;
    return _GetForwardedTargetsImpl(&visited, &uniqueTargets, targets, &foundAnyErrors, includeForwardingRels) &&
           !foundAnyErrors;
}

bool UsdRelationship::GetForwardedTargets(SdfPathVector* targets) const {
    if (!targets) {
        TF_CODING_ERROR("Passed null pointer for targets on <%s>", GetPath().GetText());
        return false;
    }
    targets->clear();
    return _GetForwardedTargets(targets,
                                /*includeForwardingRels=*/false);
}

bool UsdRelationship::HasAuthoredTargets() const {
    return HasAuthoredMetadata(SdfFieldKeys->TargetPaths);
}

SdfRelationshipSpecHandle UsdRelationship::_CreateSpec(bool fallbackCustom) const {
    UsdStage* stage = _GetStage();
    // Try to create a spec for editing either from the definition or from
    // copying existing spec info.
    TfErrorMark m;
    if (SdfRelationshipSpecHandle relSpec = stage->_CreateRelationshipSpecForEditing(*this)) {
        return relSpec;
    }

    // If creating the spec on the stage failed without issuing an error, that
    // means there was no existing authored scene description to go on (i.e. no
    // builtin info from prim type, and no existing authored spec).  Stamp a
    // spec with the provided default values.
    if (m.IsClean()) {
        SdfChangeBlock block;
        return SdfRelationshipSpec::New(stage->_CreatePrimSpecForEditing(GetPrim()), _PropName().GetString(),
                                        /* custom = */ fallbackCustom, SdfVariabilityUniform);
    }
    return TfNullPtr;
}

bool UsdRelationship::_Create(bool fallbackCustom) const {
    return bool(_CreateSpec(fallbackCustom));
}

PXR_NAMESPACE_CLOSE_SCOPE
