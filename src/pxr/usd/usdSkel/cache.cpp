//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include "pxr/usd/usdSkel/cache.h"

#include "pxr/usd/usd/primRange.h"

#include "pxr/usd/usdSkel/animation.h"
#include "pxr/usd/usdSkel/bindingAPI.h"
#include "pxr/usd/usdSkel/cacheImpl.h"
#include "pxr/usd/usdSkel/debugCodes.h"
#include "pxr/usd/usdSkel/root.h"
#include "pxr/usd/usdSkel/skeleton.h"
#include "pxr/usd/usdSkel/skeletonQuery.h"
#include "pxr/usd/usdSkel/skinningQuery.h"

#include "pxr/base/arch/hints.h"

PXR_NAMESPACE_OPEN_SCOPE

UsdSkelCache::UsdSkelCache() : _impl(new UsdSkel_CacheImpl) {}

void UsdSkelCache::Clear() {
    return UsdSkel_CacheImpl::WriteScope(_impl.get()).Clear();
}

// XXX: This method exists only so that it's clear to users that
// GetAnimQuery() is valid on UsdSkelAnimation prims.
UsdSkelAnimQuery UsdSkelCache::GetAnimQuery(const UsdSkelAnimation& anim) const {
    return UsdSkel_CacheImpl::ReadScope(_impl.get()).FindOrCreateAnimQuery(anim.GetPrim());
}

// XXX: Keeping this method around for backwards-compatibility,
/// but we should prefer the form above.
UsdSkelAnimQuery UsdSkelCache::GetAnimQuery(const UsdPrim& prim) const {
    return UsdSkel_CacheImpl::ReadScope(_impl.get()).FindOrCreateAnimQuery(prim);
}

UsdSkelSkeletonQuery UsdSkelCache::GetSkelQuery(const UsdSkelSkeleton& skel) const {
    return UsdSkel_CacheImpl::ReadScope(_impl.get()).FindOrCreateSkelQuery(skel.GetPrim());
}

bool UsdSkelCache::Populate(const UsdSkelRoot& root, Usd_PrimFlagsPredicate predicate) const {
    return UsdSkel_CacheImpl::ReadScope(_impl.get()).Populate(root, predicate);
}

UsdSkelSkinningQuery UsdSkelCache::GetSkinningQuery(const UsdPrim& prim) const {
    return UsdSkel_CacheImpl::ReadScope(_impl.get()).GetSkinningQuery(prim);
}

namespace {

struct _CompareSkels {
    bool operator()(const UsdSkelSkeleton& a, const UsdSkelSkeleton& b) const { return a.GetPrim() < b.GetPrim(); }
};

}  // namespace

bool UsdSkelCache::ComputeSkelBindings(const UsdSkelRoot& skelRoot,
                                       std::vector<UsdSkelBinding>* bindings,
                                       Usd_PrimFlagsPredicate predicate) const {
    TRACE_FUNCTION();

    if (!skelRoot) {
        TF_CODING_ERROR("'skelRoot' is invalid.");
        return false;
    }

    if (!bindings) {
        TF_CODING_ERROR("'bindings' pointer is null.");
        return false;
    }

    TF_DEBUG(USDSKEL_CACHE)
            .Msg("[UsdSkelCache] Compute skel bindings for <%s>\n", skelRoot.GetPrim().GetPath().GetText());

    bindings->clear();

    std::map<UsdSkelSkeleton, VtArray<UsdSkelSkinningQuery>, _CompareSkels> bindingMap;

    // Traverse over the prims beneath the skelRoot.
    // While traversing, we maintain a stack of 'bound' skeletons,
    // and map the last item on the stack to descendant prims.
    // This is done to handle inherited skel:skeleton bindings.

    std::vector<UsdSkelSkeleton> skelStack(1);

    const auto range = UsdPrimRange::PreAndPostVisit(skelRoot.GetPrim(), predicate);
    for (auto it = range.begin(); it != range.end(); ++it) {
        if (ARCH_UNLIKELY(!it->IsA<UsdGeomImageable>())) {
            if (!it.IsPostVisit()) {
                TF_DEBUG(USDSKEL_CACHE)
                        .Msg("[UsdSkelCache]  Pruning traversal at <%s> "
                             "(prim is not UsdGeomImageable)\n",
                             it->GetPath().GetText());

                it.PruneChildren();
            }
            continue;
        }

        if (it.IsPostVisit()) {
            if (TF_VERIFY(!skelStack.empty())) {
                skelStack.pop_back();
            } else {
                return false;
            }
            continue;
        }

        const UsdSkelBindingAPI binding(*it);

        UsdSkelSkeleton skel;
        if (!(it->HasAPI<UsdSkelBindingAPI>() && binding.GetSkeleton(&skel))) {
            skel = skelStack.back();
        } else {
            TF_DEBUG(USDSKEL_CACHE)
                    .Msg("[UsdSkelCache]  Found skel binding at <%s> "
                         "which targets skel <%s>.\n",
                         it->GetPath().GetText(), skel.GetPrim().GetPath().GetText());
        }

        if (skel && skel.GetPrim().IsActive()) {
            if (const UsdSkelSkinningQuery query = GetSkinningQuery(*it)) {
                TF_DEBUG(USDSKEL_CACHE)
                        .Msg("[UsdSkelCache]  Found skinnable prim <%s>, bound to "
                             "skel <%s>.\n",
                             it->GetPath().GetText(), skel.GetPrim().GetPath().GetText());

                bindingMap[skel].push_back(query);

                // Don't allow skinnable prims to be nested.
                it.PruneChildren();
            }
        }
        skelStack.push_back(skel);
    }

    bindings->reserve(bindingMap.size());
    for (const auto& pair : bindingMap) {
        bindings->emplace_back(pair.first, pair.second);
    }
    return true;
}

bool UsdSkelCache::ComputeSkelBinding(const UsdSkelRoot& skelRoot,
                                      const UsdSkelSkeleton& skel,
                                      UsdSkelBinding* binding,
                                      Usd_PrimFlagsPredicate predicate) const {
    TRACE_FUNCTION();

    if (!skelRoot) {
        TF_CODING_ERROR("'skelRoot' is invalid.");
        return false;
    }

    if (!skel) {
        TF_CODING_ERROR("'skel' is invalid.");
        return false;
    }

    if (!binding) {
        TF_CODING_ERROR("'binding' pointer is null.");
        return false;
    }

    // Traverse over the prims beneath the skelRoot.
    // While traversing, we maintain a stack of 'bound' skeletons,
    // and map the last item on the stack to descendant prims.
    // This is done to handle inherited skel bindings.

    std::vector<UsdSkelSkeleton> skelStack(1);
    VtArray<UsdSkelSkinningQuery> skinningQueries;

    const auto range = UsdPrimRange::PreAndPostVisit(skelRoot.GetPrim(), predicate);
    for (auto it = range.begin(); it != range.end(); ++it) {
        if (ARCH_UNLIKELY(!it->IsA<UsdGeomImageable>())) {
            if (!it.IsPostVisit()) {
                TF_DEBUG(USDSKEL_CACHE)
                        .Msg("[UsdSkelCache]  Pruning traversal at <%s> "
                             "(prim is not UsdGeomImageable)\n",
                             it->GetPath().GetText());

                it.PruneChildren();
            }
            continue;
        }

        if (it.IsPostVisit()) {
            if (TF_VERIFY(!skelStack.empty())) {
                skelStack.pop_back();
            } else {
                return false;
            }
            continue;
        }

        const UsdSkelBindingAPI binding(*it);

        UsdSkelSkeleton boundSkel;
        if (!(it->HasAPI<UsdSkelBindingAPI>() && binding.GetSkeleton(&boundSkel))) {
            boundSkel = skelStack.back();
        } else {
            TF_DEBUG(USDSKEL_CACHE)
                    .Msg("[UsdSkelCache]  Found skel binding at <%s> "
                         "which targets skel <%s>.\n",
                         it->GetPath().GetText(), boundSkel.GetPrim().GetPath().GetText());
        }

        if (boundSkel.GetPrim() == skel.GetPrim()) {
            if (const UsdSkelSkinningQuery query = GetSkinningQuery(*it)) {
                TF_DEBUG(USDSKEL_CACHE).Msg("[UsdSkelCache]  Found skinnable prim <%s>\n", it->GetPath().GetText());

                skinningQueries.push_back(query);

                // Don't allow skinnable prims to be nested.
                it.PruneChildren();
            }
        }
        skelStack.push_back(boundSkel);
    }

    *binding = UsdSkelBinding(skel, skinningQueries);
    return true;
}

PXR_NAMESPACE_CLOSE_SCOPE
