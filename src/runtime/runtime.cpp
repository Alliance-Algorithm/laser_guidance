#include "laser_guidance/runtime.hpp"

#include <memory>
#include <utility>

#include "runtime/runtime_base.hpp"

namespace rmcs_laser_guidance {

struct CompetitionRuntime::Impl {
    explicit Impl(Config config, RecordSessionOptions record_options)
        : runtime(std::move(config), std::move(record_options)) {}

    runtime_internal::CompetitionRuntimeAdapter runtime;
};

CompetitionRuntime::CompetitionRuntime(Config config, RecordSessionOptions record_options)
    : impl_(std::make_unique<Impl>(std::move(config), std::move(record_options))) {}

CompetitionRuntime::~CompetitionRuntime() = default;

auto CompetitionRuntime::start() -> std::expected<void, std::string> { return impl_->runtime.start(); }
auto CompetitionRuntime::stop() -> void { impl_->runtime.stop(); }
auto CompetitionRuntime::join() -> void { impl_->runtime.join(); }
auto CompetitionRuntime::submit_command(const RuntimeCommand& command)
    -> std::expected<void, std::string> {
    return impl_->runtime.submit_command(command);
}
auto CompetitionRuntime::snapshot() const -> RuntimeSnapshot { return impl_->runtime.snapshot(); }

struct PreviewRuntime::Impl {
    explicit Impl(Config config)
        : runtime(std::move(config)) {}

    runtime_internal::PreviewRuntimeAdapter runtime;
};

PreviewRuntime::PreviewRuntime(Config config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

PreviewRuntime::~PreviewRuntime() = default;

auto PreviewRuntime::start() -> std::expected<void, std::string> { return impl_->runtime.start(); }
auto PreviewRuntime::stop() -> void { impl_->runtime.stop(); }
auto PreviewRuntime::join() -> void { impl_->runtime.join(); }
auto PreviewRuntime::submit_command(const RuntimeCommand& command)
    -> std::expected<void, std::string> {
    return impl_->runtime.submit_command(command);
}
auto PreviewRuntime::snapshot() const -> RuntimeSnapshot { return impl_->runtime.snapshot(); }

} // namespace rmcs_laser_guidance
