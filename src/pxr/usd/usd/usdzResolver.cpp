//
// Copyright 2018 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include "pxr/pxr.h"

#include "pxr/usd/usd/usdzResolver.h"
#include "pxr/usd/usd/zipFile.h"

#include "pxr/usd/ar/asset.h"
#include "pxr/usd/ar/definePackageResolver.h"
#include "pxr/usd/ar/resolvedPath.h"
#include "pxr/usd/ar/resolver.h"

#include <tbb/concurrent_hash_map.h>

#include <memory>

PXR_NAMESPACE_OPEN_SCOPE

Usd_UsdzResolverCache& Usd_UsdzResolverCache::GetInstance() {
    static Usd_UsdzResolverCache cache;
    return cache;
}

Usd_UsdzResolverCache::Usd_UsdzResolverCache() {}

struct Usd_UsdzResolverCache::_Cache {
    using _Map = tbb::concurrent_hash_map<std::string, AssetAndZipFile>;
    _Map _pathToEntryMap;
};

void Usd_UsdzResolverCache::BeginCacheScope(VtValue* cacheScopeData) {
    _caches.BeginCacheScope(cacheScopeData);
}

void Usd_UsdzResolverCache::EndCacheScope(VtValue* cacheScopeData) {
    _caches.EndCacheScope(cacheScopeData);
}

Usd_UsdzResolverCache::_CachePtr Usd_UsdzResolverCache::_GetCurrentCache() {
    return _caches.GetCurrentCache();
}

Usd_UsdzResolverCache::AssetAndZipFile Usd_UsdzResolverCache::_OpenZipFile(const std::string& path) {
    AssetAndZipFile result;
    result.first = ArGetResolver().OpenAsset(ArResolvedPath(path));
    if (result.first) {
        result.second = UsdZipFile::Open(result.first);
    }
    return result;
}

Usd_UsdzResolverCache::AssetAndZipFile Usd_UsdzResolverCache::FindOrOpenZipFile(const std::string& packagePath) {
    _CachePtr currentCache = _GetCurrentCache();
    if (currentCache) {
        _Cache::_Map::accessor accessor;
        if (currentCache->_pathToEntryMap.insert(accessor, std::make_pair(packagePath, AssetAndZipFile()))) {
            accessor->second = _OpenZipFile(packagePath);
        }
        return accessor->second;
    }

    return _OpenZipFile(packagePath);
}

// ------------------------------------------------------------

AR_DEFINE_PACKAGE_RESOLVER(Usd_UsdzResolver, ArPackageResolver);

Usd_UsdzResolver::Usd_UsdzResolver() {}

void Usd_UsdzResolver::BeginCacheScope(VtValue* cacheScopeData) {
    Usd_UsdzResolverCache::GetInstance().BeginCacheScope(cacheScopeData);
}

void Usd_UsdzResolver::EndCacheScope(VtValue* cacheScopeData) {
    Usd_UsdzResolverCache::GetInstance().EndCacheScope(cacheScopeData);
}

std::string Usd_UsdzResolver::Resolve(const std::string& packagePath, const std::string& packagedPath) {
    std::shared_ptr<ArAsset> asset;
    UsdZipFile zipFile;
    std::tie(asset, zipFile) = Usd_UsdzResolverCache::GetInstance().FindOrOpenZipFile(packagePath);

    if (!zipFile) {
        return std::string();
    }
    return zipFile.Find(packagedPath) != zipFile.end() ? packagedPath : std::string();
}

namespace {

class _Asset : public ArAsset {
private:
    std::shared_ptr<ArAsset> _sourceAsset;
    UsdZipFile _zipFile;
    const char* _dataInZipFile;
    size_t _offsetInZipFile;
    size_t _sizeInZipFile;

public:
    explicit _Asset(std::shared_ptr<ArAsset>&& sourceAsset,
                    UsdZipFile&& zipFile,
                    const char* dataInZipFile,
                    size_t offsetInZipFile,
                    size_t sizeInZipFile)
        : _sourceAsset(std::move(sourceAsset)),
          _zipFile(std::move(zipFile)),
          _dataInZipFile(dataInZipFile),
          _offsetInZipFile(offsetInZipFile),
          _sizeInZipFile(sizeInZipFile) {}

    size_t GetSize() const override { return _sizeInZipFile; }

    std::shared_ptr<const char> GetBuffer() const override {
        struct _Deleter {
            void operator()(const char* b) { zipFile = UsdZipFile(); }
            UsdZipFile zipFile;
        };

        _Deleter d;
        d.zipFile = _zipFile;

        return std::shared_ptr<const char>(_dataInZipFile, d);
    }

    size_t Read(void* buffer, size_t count, size_t offset) const override {
        if (ARCH_UNLIKELY(offset + count > _sizeInZipFile)) {
            return 0;
        }
        memcpy(buffer, _dataInZipFile + offset, count);
        return count;
    }

    std::pair<FILE*, size_t> GetFileUnsafe() const override {
        std::pair<FILE*, size_t> result = _sourceAsset->GetFileUnsafe();
        if (result.first) {
            result.second += _offsetInZipFile;
        }
        return result;
    }
};

}  // end anonymous namespace

std::shared_ptr<ArAsset> Usd_UsdzResolver::OpenAsset(const std::string& packagePath, const std::string& packagedPath) {
    std::shared_ptr<ArAsset> asset;
    UsdZipFile zipFile;
    std::tie(asset, zipFile) = Usd_UsdzResolverCache::GetInstance().FindOrOpenZipFile(packagePath);

    if (!zipFile) {
        return nullptr;
    }

    auto iter = zipFile.Find(packagedPath);
    if (iter == zipFile.end()) {
        return nullptr;
    }

    const UsdZipFile::FileInfo info = iter.GetFileInfo();

    if (info.compressionMethod != 0) {
        TF_RUNTIME_ERROR("Cannot open %s in %s: compressed files are not supported", packagedPath.c_str(),
                         packagePath.c_str());
        return nullptr;
    }

    if (info.encrypted) {
        TF_RUNTIME_ERROR("Cannot open %s in %s: encrypted files are not supported", packagedPath.c_str(),
                         packagePath.c_str());
        return nullptr;
    }

    return std::shared_ptr<ArAsset>(
            new _Asset(std::move(asset), std::move(zipFile), iter.GetFile(), info.dataOffset, info.size));
}

PXR_NAMESPACE_CLOSE_SCOPE
