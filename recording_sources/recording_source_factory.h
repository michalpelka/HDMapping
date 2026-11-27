#pragma once

#include "recording_source.h"

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

//! Options form a recording kind shows before it opens (e.g. bag topics). The GUI owns the window; draw() fills it.
class RecordingSourceConfigUi
{
public:
    virtual ~RecordingSourceConfigUi() = default;

    //! Title of the window that holds the form.
    virtual std::string title() const = 0;
    //! Draws the form into the current ImGui window. Returns true on the frame the user asks to open the recording.
    virtual bool draw() = 0;
    //! Opens the recording with the chosen options. Returns null and sets error when it cannot.
    virtual std::shared_ptr<RecordingSource> open(std::string& error) const = 0;
};

//! One kind of recording (Mandeye LAZ/CSV, ROS1 bag, MCAP, ...): recognises its inputs and opens them as a RecordingSource.
//! Every kind registers itself in one per-process registry, which the static members reach; the apps choose through
//! find() and never name a concrete source.
class RecordingSourceFactory
{
public:
    virtual ~RecordingSourceFactory() = default;

    //! Name of the kind, for logs and dialogs.
    virtual std::string name() const = 0;
    //! Kinds with a higher priority are asked first when several accept the same paths.
    virtual int priority() const = 0;
    //! True when this kind reads paths (files and folders as the user picked or dropped them).
    virtual bool accepts(const std::vector<std::filesystem::path>& paths) const = 0;
    //! Opens paths with default options. Returns null and sets error when it cannot.
    virtual std::shared_ptr<RecordingSource> open(const std::vector<std::filesystem::path>& paths, std::string& error) const = 0;
    //! Options form to show before opening paths; null when the kind needs none and open() can be called directly.
    virtual std::unique_ptr<RecordingSourceConfigUi> configure(const std::vector<std::filesystem::path>& paths) const = 0;
    //! File dialog filter pairs (description, patterns) of the files this kind opens; empty for folder-only kinds.
    virtual std::vector<std::string> fileFilter() const = 0;

    //! Adds a kind to the registry. Thread safe; usually called through HDMAPPING_REGISTER_RECORDING_SOURCE.
    static void registerFactory(std::shared_ptr<const RecordingSourceFactory> factory);
    //! Every registered kind, highest priority first.
    static std::vector<std::shared_ptr<const RecordingSourceFactory>> factories();
    //! The highest-priority kind that accepts paths; null when none does.
    static std::shared_ptr<const RecordingSourceFactory> find(const std::vector<std::filesystem::path>& paths);
    //! Filter pairs for a file dialog that opens any registered kind: one entry for all of them, one per kind, then
    //! "All files".
    static std::vector<std::string> fileFilters();
};

//! Adapts the static members of Source to RecordingSourceFactory. Source provides
//!   static std::string name();
//!   static bool accepts(const std::vector<std::filesystem::path>& paths);
//!   static std::shared_ptr<RecordingSource> open(const std::vector<std::filesystem::path>& paths, std::string& error);
//! and optionally
//!   static int priority();                                                                 // default 0
//!   static std::unique_ptr<RecordingSourceConfigUi> configure(const std::vector<std::filesystem::path>& paths);
//!   static std::vector<std::string> fileFilter();
template<typename Source>
class RecordingSourceFactoryFor final : public RecordingSourceFactory
{
public:
    std::string name() const override
    {
        return Source::name();
    }

    int priority() const override
    {
        if constexpr (requires { Source::priority(); })
            return Source::priority();
        else
            return 0;
    }

    bool accepts(const std::vector<std::filesystem::path>& paths) const override
    {
        return Source::accepts(paths);
    }

    std::shared_ptr<RecordingSource> open(const std::vector<std::filesystem::path>& paths, std::string& error) const override
    {
        return Source::open(paths, error);
    }

    std::unique_ptr<RecordingSourceConfigUi> configure(const std::vector<std::filesystem::path>& paths) const override
    {
        if constexpr (requires { Source::configure(paths); })
            return Source::configure(paths);
        else
            return nullptr;
    }

    std::vector<std::string> fileFilter() const override
    {
        if constexpr (requires { Source::fileFilter(); })
            return Source::fileFilter();
        else
            return {};
    }
};

#define HDMAPPING_RECORDING_SOURCE_CONCAT_(a, b) a##b
#define HDMAPPING_RECORDING_SOURCE_CONCAT(a, b) HDMAPPING_RECORDING_SOURCE_CONCAT_(a, b)

//! Registers the recording kind Source (see RecordingSourceFactoryFor) when the program, or the DLL holding this line,
//! starts. Use it once per kind at namespace scope in the kind's .cpp. That .cpp must be compiled into the executable or
//! DLL: in a static library the linker drops the unreferenced registration and the kind silently never appears.
#define HDMAPPING_REGISTER_RECORDING_SOURCE(Source)                                                                                        \
    namespace                                                                                                                              \
    {                                                                                                                                      \
        [[maybe_unused]] const bool HDMAPPING_RECORDING_SOURCE_CONCAT(recording_source_registered_, __LINE__) =                            \
            (RecordingSourceFactory::registerFactory(std::make_shared<RecordingSourceFactoryFor<Source>>()), true);                        \
    }
