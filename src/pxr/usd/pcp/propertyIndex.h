//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#ifndef PXR_USD_PCP_PROPERTY_INDEX_H
#define PXR_USD_PCP_PROPERTY_INDEX_H

#include "pxr/pxr.h"
#include "pxr/usd/pcp/api.h"
#include "pxr/usd/pcp/errors.h"
#include "pxr/usd/pcp/iterator.h"
#include "pxr/usd/pcp/node.h"

#include "pxr/usd/sdf/path.h"
#include "pxr/usd/sdf/propertySpec.h"

#include <memory>
#include <vector>

PXR_NAMESPACE_OPEN_SCOPE

// Forward declarations:
class PcpCache;

/// \class Pcp_PropertyInfo
///
/// Private helper structure containing information about a property in the
/// property stack.
///
struct Pcp_PropertyInfo {
    Pcp_PropertyInfo() {}
    Pcp_PropertyInfo(const SdfPropertySpecHandle& prop, const PcpNodeRef& node)
        : propertySpec(prop), originatingNode(node) {}

    SdfPropertySpecHandle propertySpec;
    PcpNodeRef originatingNode;
};

/// \class PcpPropertyIndex
///
/// PcpPropertyIndex is an index of all sites in scene description that
/// contribute opinions to a specific property, under composition
/// semantics.
///
class PcpPropertyIndex {
public:
    /// Construct an empty property index.
    PCP_API
    PcpPropertyIndex();

    /// Copy-construct a property index.
    PCP_API
    PcpPropertyIndex(const PcpPropertyIndex& rhs);

    /// Swap the contents of this property index with \p index.
    PCP_API
    void Swap(PcpPropertyIndex& index);

    /// Returns true if this property index contains no opinions, false
    /// otherwise.
    PCP_API
    bool IsEmpty() const;

    /// Returns range of iterators that encompasses properties in this
    /// index's property stack.
    ///
    /// By default, this returns a range encompassing all properties in the
    /// index. If \p localOnly is specified, the range will only include
    /// properties from local nodes in its owning prim's graph.
    PCP_API
    PcpPropertyRange GetPropertyRange(bool localOnly = false) const;

    /// Return the list of errors local to this property.
    PcpErrorVector GetLocalErrors() const { return _localErrors ? *_localErrors.get() : PcpErrorVector(); }

    /// Returns the number of local properties in this prim index.
    PCP_API
    size_t GetNumLocalSpecs() const;

private:
    friend class PcpPropertyIterator;
    friend class Pcp_PropertyIndexer;

    // The property stack is a list of Pcp_PropertyInfo objects in
    // strong-to-weak order.
    std::vector<Pcp_PropertyInfo> _propertyStack;

    /// List of errors local to this property, encountered during computation.
    /// NULL if no errors were found (the expected common case).
    std::unique_ptr<PcpErrorVector> _localErrors;
};

/// Builds a property index for the property at \p path,
/// internally computing and caching an owning prim index as necessary.
/// \p allErrors will contain any errors encountered.
PCP_API
void PcpBuildPropertyIndex(const SdfPath& propertyPath,
                           PcpCache* cache,
                           PcpPropertyIndex* propertyIndex,
                           PcpErrorVector* allErrors);

/// Builds a prim property index for the property at \p propertyPath.
/// \p allErrors will contain any errors encountered.
PCP_API
void PcpBuildPrimPropertyIndex(const SdfPath& propertyPath,
                               const PcpCache& cache,
                               const PcpPrimIndex& owningPrimIndex,
                               PcpPropertyIndex* propertyIndex,
                               PcpErrorVector* allErrors);

PXR_NAMESPACE_CLOSE_SCOPE

#endif  // PXR_USD_PCP_PROPERTY_INDEX_H
