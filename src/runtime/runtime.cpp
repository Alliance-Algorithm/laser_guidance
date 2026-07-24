#include "laser_guidance/runtime.hpp"

#include <memory>
#include <utility>

#include "laser_guidance/error.hpp"
#include "runtime/control_loop.hpp"

namespace rmcs_laser_guidance {

struct CompetitionRuntime::Impl {
    explicit Impl(Config config, CompetitionRuntimeOptions options)
        : config(std::move(config))
        , options(std::move(options))
        , runtime(std::make_unique<runtime_internal::ControlLoop>(this->config, this->options)) {}

    Config config;
    CompetitionRuntimeOptions options;
    std::unique_ptr<runtime_internal::ControlLoop> runtime{};
};

CompetitionRuntime::CompetitionRuntime(Config config, CompetitionRuntimeOptions options)
    : impl_(std::make_unique<Impl>(std::move(config), std::move(options))) {}

CompetitionRuntime::~CompetitionRuntime() = default;

auto CompetitionRuntime::start() -> std::expected<void, Error> {
    return impl_->runtime->start();
}
auto CompetitionRuntime::run() -> std::expected<void, Error> {
    return impl_->runtime->run();
}
auto CompetitionRuntime::stop() -> void { impl_->runtime->stop(); }
auto CompetitionRuntime::join() -> void { impl_->runtime->join(); }
auto CompetitionRuntime::submit_command(const RuntimeCommand& command)
    -> std::expected<void, Error> {
    return impl_->runtime->submit_command(command);
}
auto CompetitionRuntime::snapshot() const -> RuntimeSnapshot {
    return impl_->runtime ? impl_->runtime->snapshot() : RuntimeSnapshot{};
}

} // namespace rmcs_laser_guidance
