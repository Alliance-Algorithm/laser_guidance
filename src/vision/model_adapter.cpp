#include "vision/model_adapter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "core/frame_format.hpp"
#include "vision/model_runtime.hpp"

namespace rmcs_laser_guidance {
namespace {

    constexpr float kConfidenceThreshold = 0.25F;
    constexpr float kNmsIouThreshold     = 0.45F;
    constexpr float kMaxAspectRatio      = 1.5F;

    struct DecodedDetection {
        float score = 0.0F;
        std::int32_t class_id = -1;
        cv::Rect2f bbox { };
    };

    auto format_tensor_metadata(const ModelTensorData& tensor) -> std::string {
        std::string metadata = "name=" + tensor.name + " shape=[";
        for (std::size_t index = 0; index < tensor.shape.size(); ++index) {
            if (index != 0) metadata += ", ";
            metadata += std::to_string(tensor.shape[index]);
        }
        metadata += "] dtype=" + tensor.element_type;
        return metadata;
    }

    auto tensor_shape_element_count(const std::vector<std::int64_t>& shape) -> std::size_t {
        std::size_t count = 1;
        for (const auto dim : shape) {
            if (dim <= 0) throw std::runtime_error("tensor shape contains non-positive dimension");
            count *= static_cast<std::size_t>(dim);
        }
        return count;
    }

    auto clip_box_to_frame(const cv::Rect2f& bbox, const Frame& frame)
        -> std::optional<cv::Rect2f> {
        if (frame.image.empty()) return std::nullopt;

        const float max_x = static_cast<float>(frame.image.cols);
        const float max_y = static_cast<float>(frame.image.rows);
        const float x1    = std::clamp(bbox.x, 0.0F, max_x);
        const float y1    = std::clamp(bbox.y, 0.0F, max_y);
        const float x2    = std::clamp(bbox.x + bbox.width, 0.0F, max_x);
        const float y2    = std::clamp(bbox.y + bbox.height, 0.0F, max_y);
        if (x2 <= x1 || y2 <= y1) return std::nullopt;

        return cv::Rect2f { x1, y1, x2 - x1, y2 - y1 };
    }

    auto unletterbox_box(const cv::Rect2f& bbox, const ModelImageTransform& transform,
        const Frame& frame) -> std::optional<cv::Rect2f> {
        if (transform.scale <= 0.0F) return std::nullopt;

        const float x1 = (bbox.x - transform.pad_x) / transform.scale;
        const float y1 = (bbox.y - transform.pad_y) / transform.scale;
        const float x2 = (bbox.x + bbox.width - transform.pad_x) / transform.scale;
        const float y2 = (bbox.y + bbox.height - transform.pad_y) / transform.scale;
        return clip_box_to_frame(cv::Rect2f { x1, y1, x2 - x1, y2 - y1 }, frame);
    }

    auto iou(const cv::Rect2f& lhs, const cv::Rect2f& rhs) -> float {
        const float x1 = std::max(lhs.x, rhs.x);
        const float y1 = std::max(lhs.y, rhs.y);
        const float x2 = std::min(lhs.x + lhs.width, rhs.x + rhs.width);
        const float y2 = std::min(lhs.y + lhs.height, rhs.y + rhs.height);
        if (x2 <= x1 || y2 <= y1) return 0.0F;

        const float intersection = (x2 - x1) * (y2 - y1);
        const float union_area   = lhs.area() + rhs.area() - intersection;
        if (union_area <= 0.0F) return 0.0F;
        return intersection / union_area;
    }

    auto apply_nms(std::vector<DecodedDetection> detections) -> std::vector<DecodedDetection> {
        std::sort(detections.begin(), detections.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.score > rhs.score; });

        std::vector<DecodedDetection> kept;
        kept.reserve(detections.size());
        for (const auto& candidate : detections) {
            bool suppressed = false;
            for (const auto& accepted : kept) {
                if (iou(candidate.bbox, accepted.bbox) > kNmsIouThreshold) {
                    suppressed = true;
                    break;
                }
            }
            if (!suppressed) kept.push_back(candidate);
        }
        return kept;
    }

    auto contour_from_bbox(const cv::Rect2f& bbox) -> std::vector<cv::Point2f> {
        return {
            { bbox.x, bbox.y },
            { bbox.x + bbox.width, bbox.y },
            { bbox.x + bbox.width, bbox.y + bbox.height },
            { bbox.x, bbox.y + bbox.height },
        };
    }

    auto brightness_for_bbox(const Frame& frame, const cv::Rect2f& bbox) -> float {
        const auto clipped = clip_box_to_frame(bbox, frame);
        if (!clipped) return 0.0F;

        cv::Mat gray = to_gray_image(frame.image);
        if (gray.empty()) return 0.0F;

        const cv::Rect roi(static_cast<int>(std::floor(clipped->x)),
            static_cast<int>(std::floor(clipped->y)), static_cast<int>(std::ceil(clipped->width)),
            static_cast<int>(std::ceil(clipped->height)));
        const cv::Rect valid_roi = roi & cv::Rect(0, 0, gray.cols, gray.rows);
        if (valid_roi.width <= 0 || valid_roi.height <= 0) return 0.0F;

        double max_value = 0.0;
        cv::minMaxLoc(gray(valid_roi), nullptr, &max_value);
        return static_cast<float>(max_value);
    }

    auto candidate_from_detection(const DecodedDetection& detection) -> ModelCandidate {
        return ModelCandidate {
            .score    = detection.score,
            .class_id = detection.class_id,
            .bbox     = detection.bbox,
            .center =
                cv::Point2f {
                    detection.bbox.x + detection.bbox.width * 0.5F,
                    detection.bbox.y + detection.bbox.height * 0.5F,
                },
        };
    }

    auto success_result(const Frame& frame, std::vector<DecodedDetection> detections,
        std::string message = { }) -> ModelAdapterResult {
        ModelAdapterResult result;
        result.success            = true;
        result.contract_supported = true;
        result.message            = std::move(message);

        detections.erase(
            std::remove_if(detections.begin(), detections.end(),
                [](const DecodedDetection& d) {
                    const float w = d.bbox.width;
                    const float h = d.bbox.height;
                    if (w <= 0.0F || h <= 0.0F) return true;
                    const float ratio = std::max(w / h, h / w);
                    return ratio > kMaxAspectRatio;
                }),
            detections.end());

        std::sort(detections.begin(), detections.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.score > rhs.score; });

        result.candidates.reserve(detections.size());
        for (const auto& detection : detections)
            result.candidates.push_back(candidate_from_detection(detection));

        if (!result.candidates.empty()) {
            const auto& top               = result.candidates.front();
            result.observation.detected   = true;
            result.observation.center     = top.center;
            result.observation.contour    = contour_from_bbox(top.bbox);
            result.observation.brightness = brightness_for_bbox(frame, top.bbox);
        }

        return result;
    }

    auto failure_result(std::string message) -> ModelAdapterResult {
        ModelAdapterResult result;
        result.message = std::move(message);
        return result;
    }

    auto reshape_rows_cols(const ModelTensorData& tensor)
        -> std::optional<std::pair<std::size_t, std::size_t>> {
        if (tensor.shape.size() == 3 && tensor.shape[0] == 1 && tensor.shape[1] > 0
            && tensor.shape[2] > 0) {
            return std::pair<std::size_t, std::size_t> {
                static_cast<std::size_t>(tensor.shape[1]),
                static_cast<std::size_t>(tensor.shape[2]),
            };
        }
        if (tensor.shape.size() == 2 && tensor.shape[0] > 0 && tensor.shape[1] > 0) {
            return std::pair<std::size_t, std::size_t> {
                static_cast<std::size_t>(tensor.shape[0]),
                static_cast<std::size_t>(tensor.shape[1]),
            };
        }
        return std::nullopt;
    }

    auto looks_like_nms_rows(const ModelTensorData& tensor) -> bool {
        const auto rows_cols = reshape_rows_cols(tensor);
        if (!rows_cols) return false;

        const auto [rows, cols] = *rows_cols;
        if (cols < 6) return false;

        const std::size_t samples   = std::min<std::size_t>(rows, 16);
        std::size_t likely_nms_rows = 0;
        for (std::size_t row = 0; row < samples; ++row) {
            const float* values = tensor.values.data() + row * cols;
            if (values[2] > values[0] && values[3] > values[1] && values[4] >= 0.0F
                && values[4] <= 1.0F) {
                ++likely_nms_rows;
            }
        }

        return likely_nms_rows * 2 >= samples;
    }

    auto decode_raw_output(const Frame& frame, const ModelRunResult& run_result,
        const ModelTensorData& tensor) -> ModelAdapterResult {
        const auto rows_cols = reshape_rows_cols(tensor);
        if (!rows_cols) {
            return failure_result("YOLOv5 raw output must be [1,N,5+C] or [N,5+C] ("
                + format_tensor_metadata(tensor) + ")");
        }

        const auto [rows, cols] = *rows_cols;
        if (cols < 6) {
            return failure_result("YOLOv5 raw output last dimension must be at least 6 ("
                + format_tensor_metadata(tensor) + ")");
        }
        if (tensor_shape_element_count(tensor.shape) != rows * cols)
            return failure_result("YOLOv5 raw output size does not match its shape ("
                + format_tensor_metadata(tensor) + ")");

        std::vector<DecodedDetection> detections;
        detections.reserve(rows);
        for (std::size_t row = 0; row < rows; ++row) {
            const float* values    = tensor.values.data() + row * cols;
            float score            = values[4];
            std::int32_t class_id  = 0;
            if (cols == 6) {
                class_id = static_cast<std::int32_t>(values[5]);
            } else {
                const auto class_begin = values + 5;
                const auto class_end   = values + cols;
                const auto best_class  = std::max_element(class_begin, class_end);
                score                  = score * (*best_class);
                class_id = static_cast<std::int32_t>(std::distance(class_begin, best_class));
            }
            if (score < kConfidenceThreshold) continue;

            const float center_x = values[0];
            const float center_y = values[1];
            const float width    = values[2];
            const float height   = values[3];
            if (width <= 0.0F || height <= 0.0F) continue;

            const cv::Rect2f input_bbox {
                center_x - width * 0.5F,
                center_y - height * 0.5F,
                width,
                height,
            };
            const auto bbox = unletterbox_box(input_bbox, run_result.transform, frame);
            if (!bbox) continue;
            detections.push_back(DecodedDetection {
                .score    = score,
                .class_id = class_id,
                .bbox     = *bbox,
            });
        }

        const auto kept = apply_nms(std::move(detections));
        if (kept.empty())
            return success_result(
                frame, { }, "model contract matched, no detections after threshold/NMS");
        return success_result(frame, kept);
    }

    auto maybe_scale_normalized_box(cv::Rect2f bbox, const ModelImageTransform& transform)
        -> cv::Rect2f {
        const float max_coord =
            std::max(std::max(bbox.x, bbox.y), std::max(bbox.x + bbox.width, bbox.y + bbox.height));
        if (max_coord <= 1.5F) {
            bbox.x *= static_cast<float>(transform.input_width);
            bbox.y *= static_cast<float>(transform.input_height);
            bbox.width *= static_cast<float>(transform.input_width);
            bbox.height *= static_cast<float>(transform.input_height);
        }
        return bbox;
    }

    auto decode_nms_rows(const Frame& frame, const ModelRunResult& run_result,
        const ModelTensorData& tensor) -> ModelAdapterResult {
        const auto rows_cols = reshape_rows_cols(tensor);
        if (!rows_cols) {
            return failure_result("YOLOv5 NMS output must be [1,N,6/7] or [N,6/7] ("
                + format_tensor_metadata(tensor) + ")");
        }

        const auto [rows, cols] = *rows_cols;
        if (cols != 6 && cols != 7)
            return failure_result("YOLOv5 NMS output last dimension must be 6 or 7 ("
                + format_tensor_metadata(tensor) + ")");

        std::vector<DecodedDetection> detections;
        detections.reserve(rows);
        for (std::size_t row = 0; row < rows; ++row) {
            const float* values = tensor.values.data() + row * cols;

            float x1    = 0.0F;
            float y1    = 0.0F;
            float x2    = 0.0F;
            float y2    = 0.0F;
            float score = 0.0F;
            std::int32_t class_id = 0;

            if (cols == 6) {
                x1    = values[0];
                y1    = values[1];
                x2    = values[2];
                y2    = values[3];
                score = values[4];
                class_id = static_cast<std::int32_t>(values[5]);
            } else if (values[5] >= 0.0F && values[5] <= 1.0F) {
                x1    = values[1];
                y1    = values[2];
                x2    = values[3];
                y2    = values[4];
                score = values[5];
                class_id = static_cast<std::int32_t>(values[0]);
            } else if (values[2] >= 0.0F && values[2] <= 1.0F) {
                score = values[2];
                x1    = values[3];
                y1    = values[4];
                x2    = values[5];
                y2    = values[6];
                class_id = static_cast<std::int32_t>(values[0]);
            } else {
                return failure_result("YOLOv5 NMS output with 7 columns is not a supported layout");
            }

            if (score < kConfidenceThreshold || x2 <= x1 || y2 <= y1) continue;

            cv::Rect2f input_bbox { x1, y1, x2 - x1, y2 - y1 };
            input_bbox      = maybe_scale_normalized_box(input_bbox, run_result.transform);
            const auto bbox = unletterbox_box(input_bbox, run_result.transform, frame);
            if (!bbox) continue;
            detections.push_back(DecodedDetection {
                .score    = score,
                .class_id = class_id,
                .bbox     = *bbox,
            });
        }

        if (detections.empty())
            return success_result(
                frame, { }, "model contract matched, no detections after threshold/NMS");
        return success_result(frame, detections);
    }

    auto decode_split_nms_outputs(const Frame& frame, const ModelRunResult& run_result)
        -> ModelAdapterResult {
        if (run_result.outputs.size() != 4)
            return failure_result("YOLOv5 split NMS contract requires four output tensors");

        const auto& num     = run_result.outputs[0];
        const auto& boxes   = run_result.outputs[1];
        const auto& scores  = run_result.outputs[2];
        const auto& classes = run_result.outputs[3];
        (void)classes;

        if (num.values.empty())
            return failure_result("YOLOv5 split NMS num_detections output is empty");
        if (boxes.shape.size() != 3 || boxes.shape[0] != 1 || boxes.shape[2] != 4)
            return failure_result("YOLOv5 split NMS boxes output must be [1,N,4]");
        if (scores.shape.size() != 2 && scores.shape.size() != 3)
            return failure_result("YOLOv5 split NMS scores output must be [1,N] or [1,N,1]");

        const std::size_t max_boxes       = static_cast<std::size_t>(boxes.shape[1]);
        const std::size_t detection_count = std::min<std::size_t>(
            static_cast<std::size_t>(std::max(0.0F, std::round(num.values[0]))), max_boxes);

        std::vector<DecodedDetection> detections;
        detections.reserve(detection_count);
        for (std::size_t index = 0; index < detection_count; ++index) {
            const float score = scores.values[index];
            if (score < kConfidenceThreshold) continue;

            const float* box = boxes.values.data() + index * 4;
            cv::Rect2f input_bbox { box[0], box[1], box[2] - box[0], box[3] - box[1] };
            input_bbox      = maybe_scale_normalized_box(input_bbox, run_result.transform);
            const auto bbox = unletterbox_box(input_bbox, run_result.transform, frame);
            if (!bbox) continue;
            detections.push_back(DecodedDetection {
                .score    = score,
                .class_id = static_cast<std::int32_t>(classes.values[index]),
                .bbox     = *bbox,
            });
        }

        if (detections.empty())
            return success_result(
                frame, { }, "model contract matched, no detections after threshold/NMS");
        return success_result(frame, detections);
    }

    class Yolov5ModelAdapter final : public ModelAdapter {
    public:
        auto adapt(const Frame& frame, const ModelRuntime& runtime) const
            -> ModelAdapterResult override {
            const auto run_result = runtime.run(frame.image);
            if (!run_result.success) return failure_result(run_result.message);
            return adapt_yolov5_outputs(frame, run_result);
        }
    };

} // namespace

auto adapt_yolov5_outputs(const Frame& frame, const ModelRunResult& run_result)
    -> ModelAdapterResult {
    if (!run_result.success) return failure_result(run_result.message);
    if (run_result.outputs.empty())
        return failure_result("model runtime returned no output tensors");

    if (run_result.outputs.size() == 4) return decode_split_nms_outputs(frame, run_result);

    if (run_result.outputs.size() != 1) {
        return failure_result("model output contract is unsupported (outputs="
            + std::to_string(run_result.outputs.size()) + ")");
    }

    const auto& tensor   = run_result.outputs.front();
    const auto rows_cols = reshape_rows_cols(tensor);
    if (!rows_cols) {
        return failure_result("model output contract is unsupported for the primary output shape ("
            + format_tensor_metadata(tensor) + ")");
    }

    const auto [rows, cols] = *rows_cols;
    if (rows == 0 || cols == 0) return failure_result("model output tensor is empty");
    if (cols == 6 && !looks_like_nms_rows(tensor))
        return decode_raw_output(frame, run_result, tensor);
    if (cols == 6 || cols == 7) return decode_nms_rows(frame, run_result, tensor);
    if (cols > 7) return decode_raw_output(frame, run_result, tensor);

    return failure_result(
        "model output contract is unsupported (last_dim=" + std::to_string(cols) + ")");
}

auto make_default_model_adapter() -> std::unique_ptr<ModelAdapter> {
    return std::make_unique<Yolov5ModelAdapter>();
}

} // namespace rmcs_laser_guidance
