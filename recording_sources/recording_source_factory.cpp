#include "recording_source_factory.h"

#include <algorithm>
#include <mutex>

namespace
{
    struct Registry
    {
        std::mutex mutex;
        std::vector<std::shared_ptr<const RecordingSourceFactory>> factories; //!< highest priority first
    };

    //! Created on first use, so registrations from any translation unit or DLL find it whatever their order.
    Registry& registry()
    {
        static Registry r;
        return r;
    }
} // namespace

void RecordingSourceFactory::registerFactory(std::shared_ptr<const RecordingSourceFactory> factory)
{
    if (!factory)
        return;
    auto& r = registry();
    std::scoped_lock lock(r.mutex);
    // stable: kinds of equal priority keep their registration order
    const auto at = std::upper_bound(
        r.factories.begin(),
        r.factories.end(),
        factory->priority(),
        [](int priority, const std::shared_ptr<const RecordingSourceFactory>& f)
        {
            return priority > f->priority();
        });
    r.factories.insert(at, std::move(factory));
}

std::vector<std::shared_ptr<const RecordingSourceFactory>> RecordingSourceFactory::factories()
{
    auto& r = registry();
    std::scoped_lock lock(r.mutex);
    return r.factories;
}

std::shared_ptr<const RecordingSourceFactory> RecordingSourceFactory::find(const std::vector<std::filesystem::path>& paths)
{
    if (paths.empty())
        return nullptr;
    for (const auto& factory : factories())
        if (factory->accepts(paths))
            return factory;
    return nullptr;
}

std::vector<std::string> RecordingSourceFactory::fileFilters()
{
    std::vector<std::string> perKind;
    std::string allPatterns;
    for (const auto& factory : factories())
    {
        const auto filter = factory->fileFilter();
        for (size_t i = 0; i + 1 < filter.size(); i += 2)
        {
            perKind.push_back(filter[i]);
            perKind.push_back(filter[i + 1]);
            allPatterns += (allPatterns.empty() ? "" : " ") + filter[i + 1];
        }
    }
    std::vector<std::string> filters;
    if (!allPatterns.empty())
    {
        filters.push_back("All supported recordings");
        filters.push_back(allPatterns);
    }
    filters.insert(filters.end(), perKind.begin(), perKind.end());
    filters.push_back("All files");
    filters.push_back("*");
    return filters;
}
