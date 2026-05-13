#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <expected>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <print>
#include <string>
#include <string_view>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <yaml-cpp/yaml.h>

#include "config.hpp"
#include "example_support.hpp"
#include "capture/v4l2_capture.hpp"

namespace {

constexpr int kChessboardCols   = 11;
constexpr int kChessboardRows   = 8;
constexpr float kSquareSizeMm   = 15.0F;
constexpr int kCornerRefineWin  = 11;
constexpr int kMinSamples       = 5;

constexpr int kCalibFlags = cv::CALIB_FIX_K3 | cv::CALIB_ZERO_TANGENT_DIST;

constexpr char kWindowName[] = "Camera Calibration (rmcs_laser_guidance)";

const cv::Scalar kOverlayBg   { 0, 0, 0 };
const cv::Scalar kOverlayFg   { 255, 255, 255 };
const cv::Scalar kOverlayWarn { 0, 200, 255 };
const cv::Scalar kOverlayOk   { 0, 255, 0 };

constexpr double kOverlayAlpha       = 0.55;
constexpr int kOverlayFont           = cv::FONT_HERSHEY_SIMPLEX;
constexpr double kOverlayFontScale   = 0.70;
constexpr double kHintsFontScale     = 0.50;
constexpr int kOverlayThickness      = 2;
constexpr int kHintsThickness        = 1;
constexpr int kOverlayMargin         = 10;
constexpr int kOverlayLineSpacing    = 28;
constexpr int kHintsLineSpacing      = 20;

struct CalibSample {
    std::vector<cv::Point2f> image_points;
    cv::Size image_size {};
};

struct CalibResult {
    bool valid = false;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    double rms_error = 0.0;
    cv::Size image_size {};
    int num_samples = 0;
};

struct ChessboardDetection {
    bool found = false;
    std::vector<cv::Point2f> corners;
};

struct CalibrationUiStatus {
    int num_samples = 0;
    const CalibResult* result = nullptr;
};

auto generate_chessboard_object_points(int cols, int rows, float square_mm)
    -> std::vector<cv::Point3f> {
    std::vector<cv::Point3f> points;
    points.reserve(static_cast<std::size_t>(cols) * static_cast<std::size_t>(rows));
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c)
            points.emplace_back(static_cast<float>(c) * square_mm,
                                static_cast<float>(r) * square_mm,
                                0.0F);
    return points;
}

auto try_find_chessboard(const cv::Mat& gray) -> ChessboardDetection {
    std::vector<cv::Point2f> corners;
    const cv::Size board(kChessboardCols, kChessboardRows);

    const bool found = cv::findChessboardCorners(
        gray, board, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

    if (!found) return {};

    const cv::TermCriteria criteria(
        cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);
    cv::cornerSubPix(gray, corners,
                     cv::Size(kCornerRefineWin, kCornerRefineWin),
                     cv::Size(-1, -1), criteria);

    return { .found = true, .corners = std::move(corners) };
}

auto draw_text_box(cv::Mat& img, std::string_view text, cv::Point org,
                   cv::Scalar fg = kOverlayFg,
                   double scale = kOverlayFontScale,
                   int thickness = kOverlayThickness) -> void {
    int baseline = 0;
    const cv::Size text_size =
        cv::getTextSize(std::string(text), kOverlayFont, scale, thickness, &baseline);
    const cv::Rect bg_rect(org.x, org.y - text_size.height - baseline,
                           text_size.width + kOverlayMargin,
                           text_size.height + baseline + kOverlayMargin);

    cv::Mat roi = img(bg_rect & cv::Rect(0, 0, img.cols, img.rows));
    cv::Mat overlay;
    cv::addWeighted(kOverlayBg, kOverlayAlpha, roi, 1.0 - kOverlayAlpha, 0.0, overlay);
    overlay.copyTo(roi);
    cv::putText(img, std::string(text),
                cv::Point(org.x + kOverlayMargin / 2, org.y - baseline / 2),
                kOverlayFont, scale, fg, thickness);
}

auto draw_status_overlay(cv::Mat& img, const CalibrationUiStatus& status) -> void {
    int y = kOverlayLineSpacing;
    const CalibResult& result = *status.result;

    {
        const auto text = std::format("Samples: {}/{}", status.num_samples, kMinSamples);
        const cv::Scalar color = (status.num_samples >= kMinSamples) ? kOverlayOk : kOverlayWarn;
        draw_text_box(img, text, { kOverlayMargin, y }, color);
        y += kOverlayLineSpacing;
    }

    if (result.valid) {
        const auto text = std::format("Calibrated | RMS = {:.3f} px", result.rms_error);
        draw_text_box(img, text, { kOverlayMargin, y }, kOverlayOk);
        y += kOverlayLineSpacing;

        if (!result.camera_matrix.empty()) {
            const double fx = result.camera_matrix.at<double>(0, 0);
            const double fy = result.camera_matrix.at<double>(1, 1);
            const double cx = result.camera_matrix.at<double>(0, 2);
            const double cy = result.camera_matrix.at<double>(1, 2);
            draw_text_box(img,
                          std::format("fx={:.1f} fy={:.1f} cx={:.1f} cy={:.1f}",
                                      fx, fy, cx, cy),
                          { kOverlayMargin, y }, kOverlayFg, 0.55, 1);
        }
    } else if (status.num_samples > 0) {
        draw_text_box(img, "Ready (not calibrated)", { kOverlayMargin, y }, kOverlayWarn);
    } else {
        draw_text_box(img, "Ready (no samples yet)", { kOverlayMargin, y }, kOverlayFg);
    }
}

auto draw_key_hints(cv::Mat& img) -> void {
    int y = img.rows - kHintsLineSpacing * 4;

    for (const auto& line : {
             "[c/Space] Capture    [p] Calibrate    [s] Save",
             "[l] Load YAML        [u] Undistort    [r] Reset",
             "[h] Toggle hints     [q/Esc] Quit",
         }) {
        draw_text_box(img, line, { kOverlayMargin, y },
                      kOverlayFg, kHintsFontScale, kHintsThickness);
        y += kHintsLineSpacing;
    }
}

auto run_calibration(const std::vector<CalibSample>& samples,
                     const std::vector<cv::Point3f>& object_template) -> CalibResult {
    CalibResult result;
    if (samples.empty()) return result;

    const cv::Size image_size = samples.front().image_size;
    std::vector<std::vector<cv::Point3f>> all_obj;
    std::vector<std::vector<cv::Point2f>> all_img;
    all_obj.reserve(samples.size());
    all_img.reserve(samples.size());

    for (const auto& s : samples) {
        all_obj.push_back(object_template);
        all_img.push_back(s.image_points);
    }

    cv::Mat camera = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat dist;
    std::vector<cv::Mat> rvecs, tvecs;

    const cv::TermCriteria criteria(
        cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON);

    const double rms = cv::calibrateCamera(
        all_obj, all_img, image_size,
        camera, dist, rvecs, tvecs, kCalibFlags, criteria);

    if (rms > 0.0) {
        result.valid         = true;
        result.camera_matrix = std::move(camera);
        result.dist_coeffs   = std::move(dist);
        result.rms_error     = rms;
        result.image_size    = image_size;
        result.num_samples   = static_cast<int>(samples.size());
    }

    return result;
}

auto fmt_timestamp() -> std::string {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm {};
    localtime_r(&t, &tm);
    char buf[64];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S%z", &tm);
    return buf;
}

auto save_calibration_yaml(const CalibResult& result,
                           const std::filesystem::path& path)
    -> std::expected<void, std::string> {
    if (!result.valid)
        return std::unexpected("no valid calibration data to save");

    YAML::Emitter out;
    out.SetIndent(2);
    out << YAML::Comment("Camera calibration parameters") << YAML::Newline
        << YAML::Comment("Generated by rmcs_laser_guidance camera_calibration") << YAML::Newline
        << YAML::Comment("Timestamp: " + fmt_timestamp()) << YAML::Newline;

    out << YAML::BeginMap;
    out << YAML::Key << "calibration" << YAML::Value << YAML::BeginMap;

    out << YAML::Key << "image_width" << YAML::Value << result.image_size.width;
    out << YAML::Key << "image_height" << YAML::Value << result.image_size.height;

    out << YAML::Key << "chessboard" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "cols" << YAML::Value << kChessboardCols;
    out << YAML::Key << "rows" << YAML::Value << kChessboardRows;
    out << YAML::Key << "square_size_mm" << YAML::Value << kSquareSizeMm;
    out << YAML::EndMap;

    out << YAML::Key << "camera_matrix" << YAML::Value
        << YAML::Flow << YAML::BeginSeq;
    for (int i = 0; i < result.camera_matrix.rows; ++i) {
        out << YAML::Flow << YAML::BeginSeq;
        for (int j = 0; j < result.camera_matrix.cols; ++j)
            out << result.camera_matrix.at<double>(i, j);
        out << YAML::EndSeq;
    }
    out << YAML::EndSeq;

    out << YAML::Key << "dist_coeffs" << YAML::Value
        << YAML::Flow << YAML::BeginSeq;
    for (int i = 0; i < static_cast<int>(result.dist_coeffs.total()); ++i)
        out << result.dist_coeffs.at<double>(i);
    out << YAML::EndSeq;

    out << YAML::Key << "rms_reprojection_error" << YAML::Value << result.rms_error;
    out << YAML::Key << "num_samples" << YAML::Value << result.num_samples;

    out << YAML::EndMap;
    out << YAML::EndMap;

    std::ofstream file(path);
    if (!file)
        return std::unexpected("failed to open file for writing: " + path.string());
    file << out.c_str();
    if (!file)
        return std::unexpected("failed to write calibration to: " + path.string());

    return {};
}

auto load_calibration_yaml(const std::filesystem::path& path)
    -> std::expected<CalibResult, std::string> {
    try {
    const YAML::Node yaml = YAML::LoadFile(path.string());
    const YAML::Node calib = yaml["calibration"];
    if (!calib)
        return std::unexpected("YAML missing 'calibration' key: " + path.string());

    CalibResult result;
    result.valid       = true;
    result.image_size  = cv::Size(calib["image_width"].as<int>(),
                                  calib["image_height"].as<int>());
    result.rms_error   = calib["rms_reprojection_error"].as<double>(0.0);
    result.num_samples = calib["num_samples"].as<int>(0);

    const YAML::Node mat = calib["camera_matrix"];
    if (!mat || mat.size() != 3)
        return std::unexpected("missing or malformed camera_matrix");
    result.camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i) {
        if (mat[i].size() != 3)
            return std::unexpected("camera_matrix row must have 3 columns");
        for (int j = 0; j < 3; ++j)
            result.camera_matrix.at<double>(i, j) = mat[i][j].as<double>();
    }

    const YAML::Node dc = calib["dist_coeffs"];
    if (!dc)
        return std::unexpected("missing dist_coeffs");
    const int n = static_cast<int>(dc.size());
    if (n < 4)
        return std::unexpected("dist_coeffs must have at least 4 elements");
    result.dist_coeffs = cv::Mat::zeros(1, n, CV_64F);
    for (int i = 0; i < n; ++i)
        result.dist_coeffs.at<double>(i) = dc[i].as<double>();

    return result;
    } catch (const std::exception& e) {
        return std::unexpected(e.what());
    }
}

auto resolve_args(int argc, char** argv,
                  std::filesystem::path& config_path,
                  std::filesystem::path& output_path) -> void {
    config_path = rmcs_laser_guidance::examples::default_config_path();
    output_path = "config/camera_calib.yaml";

    if (argc > 1) config_path = argv[1];
    if (argc > 2) output_path = argv[2];
}

struct FlashMessage {
    std::string text;
    std::chrono::steady_clock::time_point expiry;
    bool warn = false;
};

auto set_flash(std::vector<FlashMessage>& messages,
               std::string text, bool warn = false) -> void {
    using namespace std::chrono_literals;
    messages.push_back({ std::move(text),
                         std::chrono::steady_clock::now() + 1500ms,
                         warn });
}

auto expire_flash(std::vector<FlashMessage>& messages) -> void {
    const auto now = std::chrono::steady_clock::now();
    std::erase_if(messages,
                  [now](const auto& m) { return m.expiry <= now; });
}

auto draw_flash_messages(cv::Mat& img,
                         const std::vector<FlashMessage>& messages) -> void {
    int y = kOverlayLineSpacing * 4;
    for (const auto& m : messages) {
        draw_text_box(img, m.text, { kOverlayMargin, y },
                      m.warn ? kOverlayWarn : kOverlayOk);
        y += kOverlayLineSpacing;
    }
}

class CalibSession {
public:
    explicit CalibSession(std::vector<cv::Point3f> object_template)
        : object_template_(std::move(object_template)) { }

    auto show_hints() const noexcept -> bool { return show_hints_; }

    auto show_undistorted() const noexcept -> bool { return show_undistorted_; }

    auto result() const noexcept -> const CalibResult& { return result_; }

    auto status() const noexcept -> CalibrationUiStatus {
        return { .num_samples = static_cast<int>(samples_.size()), .result = &result_ };
    }

    auto capture(const ChessboardDetection& detection, cv::Size image_size) -> void {
        if (!detection.found) {
            flash("No chessboard detected — adjust angle/lighting", true);
            return;
        }

        samples_.push_back({ .image_points = detection.corners, .image_size = image_size });
        flash(std::format("Captured sample #{} — {} corners found",
                          samples_.size(), detection.corners.size()));
    }

    auto calibrate() -> void {
        if (static_cast<int>(samples_.size()) < kMinSamples) {
            flash(std::format("Need at least {} samples (have {})",
                              kMinSamples, samples_.size()), true);
            return;
        }

        result_ = run_calibration(samples_, object_template_);
        if (!result_.valid) {
            flash("Calibration failed — degenerate geometry?", true);
            return;
        }

        std::println("");
        std::println("--- Calibration result ---");
        std::println("RMS reprojection error : {:.4f} px", result_.rms_error);
        std::cout << "Camera matrix:\n  " << result_.camera_matrix << "\n";
        std::cout << "Distortion coefficients:\n  " << result_.dist_coeffs << "\n";
        std::println("Image size: {}x{}", result_.image_size.width, result_.image_size.height);
        std::println("Samples used: {}", result_.num_samples);
        std::println("---------------------------");
        std::println("");

        flash(std::format("Calibrated! RMS={:.3f} px — press [u] to preview",
                          result_.rms_error));
    }

    auto save(const std::filesystem::path& output_path) -> void {
        if (!result_.valid) {
            flash("No calibration data — run [p] first", true);
            return;
        }

        const auto saved = save_calibration_yaml(result_, output_path);
        if (!saved) {
            flash(std::format("Save failed: {}", saved.error()), true);
            return;
        }

        flash(std::format("Saved → {}", output_path.string()));
    }

    auto load(const std::filesystem::path& output_path) -> void {
        auto loaded = load_calibration_yaml(output_path);
        if (!loaded) {
            flash(std::format("Load failed: {}", loaded.error()), true);
            return;
        }

        result_ = std::move(*loaded);
        show_undistorted_ = true;
        flash(std::format("Loaded → {} (RMS={:.3f} px)",
                          output_path.string(), result_.rms_error));
    }

    auto toggle_undistorted() -> void {
        if (!result_.valid) {
            flash("Calibrate [p] or load [l] first", true);
            return;
        }

        show_undistorted_ = !show_undistorted_;
        flash(show_undistorted_ ? "Undistorted preview ON" : "Undistorted preview OFF");
    }

    auto toggle_hints() noexcept -> void { show_hints_ = !show_hints_; }

    auto reset() -> void {
        samples_.clear();
        result_ = CalibResult {};
        show_undistorted_ = false;
        flash("Samples cleared");
    }

    auto expire_flash_messages() -> void { expire_flash(flash_); }

    auto draw_flash(cv::Mat& img) const -> void { draw_flash_messages(img, flash_); }

private:
    auto flash(std::string text, bool warn = false) -> void {
        set_flash(flash_, std::move(text), warn);
    }

    std::vector<cv::Point3f> object_template_;
    std::vector<CalibSample> samples_;
    CalibResult result_;
    std::vector<FlashMessage> flash_;
    bool show_hints_ = true;
    bool show_undistorted_ = false;
};

enum class KeyAction {
    keep_running,
    exit,
};

auto handle_key(int key, CalibSession& session,
                const ChessboardDetection& detection, cv::Size image_size,
                const std::filesystem::path& output_path) -> KeyAction {
    using rmcs_laser_guidance::examples::should_exit_from_key;

    switch (key) {
    case 'c':
    case ' ':
        session.capture(detection, image_size);
        break;
    case 'p':
        session.calibrate();
        break;
    case 's':
        session.save(output_path);
        break;
    case 'l':
        session.load(output_path);
        break;
    case 'u':
        session.toggle_undistorted();
        break;
    case 'r':
        session.reset();
        break;
    case 'h':
        session.toggle_hints();
        break;
    default:
        if (should_exit_from_key(key)) return KeyAction::exit;
        break;
    }

    return KeyAction::keep_running;
}

} // namespace

int main(int argc, char** argv) {
    try {
        std::filesystem::path config_path;
        std::filesystem::path output_path;
        resolve_args(argc, argv, config_path, output_path);

        const auto config = rmcs_laser_guidance::load_config(config_path);

        rmcs_laser_guidance::V4l2Capture capture(config.v4l2);
        const auto open_result = capture.open();
        if (!open_result) {
            std::println(stderr, "Failed to open camera: {}", open_result.error());
            return 1;
        }

        const auto& negotiated = capture.negotiated_format();
        std::println("Camera opened: {} ({}x{} @ {} fps {})",
                     negotiated.device_path.string(),
                     negotiated.width, negotiated.height,
                     negotiated.framerate, negotiated.fourcc);

        CalibSession session(generate_chessboard_object_points(
            kChessboardCols, kChessboardRows, kSquareSizeMm));

        cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);

        while (true) {
            auto frame = capture.read_frame();
            if (!frame) {
                std::println(stderr, "Frame read error: {}", frame.error());
                continue;
            }

            cv::Mat display = frame->image.clone();

            if (session.show_undistorted() && session.result().valid)
                cv::undistort(display, display,
                              session.result().camera_matrix,
                              session.result().dist_coeffs);

            cv::Mat gray;
            cv::cvtColor(display, gray, cv::COLOR_BGR2GRAY);
            const auto detection = try_find_chessboard(gray);

            if (detection.found)
                cv::drawChessboardCorners(
                    display, cv::Size(kChessboardCols, kChessboardRows),
                    detection.corners, detection.found);

            draw_status_overlay(display, session.status());
            if (session.show_hints()) draw_key_hints(display);

            session.expire_flash_messages();
            session.draw_flash(display);

            cv::imshow(kWindowName, display);
            const int key = cv::waitKey(1);

            if (handle_key(key, session, detection, display.size(), output_path)
                == KeyAction::exit)
                break;
        }

        capture.close();
        cv::destroyWindow(kWindowName);
        return 0;

    } catch (const std::exception& e) {
        std::println(stderr, "example_camera_calibration failed: {}", e.what());
        return 1;
    }
}
