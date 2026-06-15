# syntax=docker/dockerfile:1
ARG ONNXRUNTIME_VERSION=1.26.0
ARG CMAKE_BUILD_TYPE=Release
ARG TARGETARCH
ARG TENSORRT_IMAGE=nvcr.io/nvidia/tensorrt:25.08-py3

FROM ${TENSORRT_IMAGE} AS nvidia-libs

FROM ubuntu:24.04 AS deps
ARG ONNXRUNTIME_VERSION
ARG TARGETARCH
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive TZ=Asia/Shanghai

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential gcc-14 g++-14 cmake ninja-build pkg-config \
    libopencv-dev libyaml-cpp-dev \
    libusb-1.0-0-dev \
    libavcodec-dev libavformat-dev libavutil-dev libswscale-dev \
    ffmpeg \
    python3 python3-yaml \
    ca-certificates curl wget \
    v4l-utils usbutils \
  && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-14 50 \
  && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-14 50 \
  && apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/*

RUN set -eux; \
  case "${TARGETARCH:-amd64}" in \
    amd64)   ort_arch=x64 ;; \
    arm64)   ort_arch=aarch64 ;; \
    *) echo "unsupported arch: ${TARGETARCH}"; exit 1 ;; \
  esac; \
  ort_tgz="onnxruntime-linux-${ort_arch}-${ONNXRUNTIME_VERSION}.tgz"; \
  ort_url="https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/${ort_tgz}"; \
  curl -fsSL -o "/tmp/${ort_tgz}" "${ort_url}"; \
  tar -xzf "/tmp/${ort_tgz}" -C /opt; \
  mv "/opt/onnxruntime-linux-${ort_arch}-${ONNXRUNTIME_VERSION}" /opt/onnxruntime; \
  mkdir -p /opt/onnxruntime/include/onnxruntime/core/session; \
  ln -sf ../../../onnxruntime_cxx_api.h /opt/onnxruntime/include/onnxruntime/core/session/onnxruntime_cxx_api.h; \
  rm "/tmp/${ort_tgz}"

ENV ONNXRUNTIME_ROOT=/opt/onnxruntime
RUN echo "${ONNXRUNTIME_ROOT}/lib" > /etc/ld.so.conf.d/onnxruntime.conf && ldconfig

FROM deps AS build
ARG CMAKE_BUILD_TYPE
WORKDIR /workspace/laser_guidance

# NVIDIA TensorRT + CUDA headers and stubs for build linking
COPY --from=nvidia-libs /usr/include/x86_64-linux-gnu/ /tmp/nvidia-headers/
RUN cp /tmp/nvidia-headers/NvInfer*.h /usr/include/x86_64-linux-gnu/ && rm -rf /tmp/nvidia-headers
COPY --from=nvidia-libs /usr/local/cuda/include /usr/local/cuda/include
COPY --from=nvidia-libs \
    /usr/local/cuda-13.0/targets/x86_64-linux/lib/libcudart.so.13.0.48 \
    /usr/local/cuda-13.0/targets/x86_64-linux/lib/libcudart.so.13 \
    /usr/local/cuda/lib64/
RUN ln -sf libcudart.so.13 /usr/local/cuda/lib64/libcudart.so && ldconfig

COPY --from=nvidia-libs \
    /usr/local/cuda-13.0/targets/x86_64-linux/lib/stubs/libcuda.so \
    /usr/local/cuda/lib64/stubs/

COPY --from=nvidia-libs \
    /usr/lib/x86_64-linux-gnu/libnvinfer.so.10.13.2 \
    /usr/lib/x86_64-linux-gnu/libnvinfer.so.10 \
    /usr/lib/x86_64-linux-gnu/libnvinfer.so \
    /usr/lib/x86_64-linux-gnu/
RUN ln -sf libnvinfer.so.10 /usr/lib/x86_64-linux-gnu/libnvinfer.so && ldconfig

COPY --from=nvidia-libs \
    /usr/lib/x86_64-linux-gnu/libnvonnxparser.so.10.13.2 \
    /usr/lib/x86_64-linux-gnu/libnvonnxparser.so.10 \
    /usr/lib/x86_64-linux-gnu/libnvonnxparser.so \
    /usr/lib/x86_64-linux-gnu/
RUN ln -sf libnvonnxparser.so.10 /usr/lib/x86_64-linux-gnu/libnvonnxparser.so

COPY --from=nvidia-libs \
    /usr/lib/x86_64-linux-gnu/libnvinfer_plugin.so.10.13.2 \
    /usr/lib/x86_64-linux-gnu/libnvinfer_plugin.so.10 \
    /usr/lib/x86_64-linux-gnu/libnvinfer_plugin.so \
    /usr/lib/x86_64-linux-gnu/
RUN ln -sf libnvinfer_plugin.so.10 /usr/lib/x86_64-linux-gnu/libnvinfer_plugin.so && ldconfig

COPY vendor/ft4222/lib/libft4222.so.1.4.4.232 /opt/libft4222/libft4222.so.1.4.4.232
RUN ln -sf libft4222.so.1.4.4.232 /opt/libft4222/libft4222.so.1 && \
    ln -sf libft4222.so.1           /opt/libft4222/libft4222.so && \
    echo "/opt/libft4222" > /etc/ld.so.conf.d/libft4222.conf && ldconfig

COPY . .
RUN rm -rf build && \
    cmake -S . -B build -G Ninja \
      -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}" \
      -DONNXRUNTIME_ROOT="${ONNXRUNTIME_ROOT}" \
      -DCUDA_LIBRARY=/usr/local/cuda/lib64/libcudart.so \
      -DCUDA_RT_LIBRARY=/usr/local/cuda/lib64/stubs/libcuda.so \
    && cmake --build build --parallel \
    && mkdir -p /opt/laser_guidance/bin \
    && find build -maxdepth 1 -type f -executable -exec cp {} /opt/laser_guidance/bin/ \;

FROM ubuntu:24.04 AS runtime
ARG ONNXRUNTIME_VERSION
ENV DEBIAN_FRONTEND=noninteractive TZ=Asia/Shanghai
ENV LASER_HEADLESS=1 NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=compute,video,utility

RUN apt-get update && apt-get install -y --no-install-recommends \
    libopencv-calib3d406t64 libopencv-core406t64 libopencv-highgui406t64 \
    libopencv-imgcodecs406t64 libopencv-imgproc406t64 libopencv-videoio406t64 \
    libyaml-cpp0.8 libusb-1.0-0 \
    ffmpeg python3 python3-yaml v4l-utils usbutils tini \
  && apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/*

COPY --from=deps /opt/onnxruntime/lib/libonnxruntime.so.1.26.0 \
                /opt/onnxruntime/lib/libonnxruntime_providers_shared.so \
                /usr/local/lib/
RUN ln -sf libonnxruntime.so.1.26.0 /usr/local/lib/libonnxruntime.so.1 && \
    ln -sf libonnxruntime.so.1      /usr/local/lib/libonnxruntime.so && \
    ldconfig

COPY --from=nvidia-libs \
    /usr/local/cuda-13.0/targets/x86_64-linux/lib/libcudart.so.13.0.48 \
    /usr/local/cuda/lib64/
RUN ln -sf libcudart.so.13.0.48 /usr/local/cuda/lib64/libcudart.so.13 && \
    ln -sf libcudart.so.13       /usr/local/cuda/lib64/libcudart.so && \
    echo /usr/local/cuda/lib64 > /etc/ld.so.conf.d/cuda.conf && ldconfig

COPY --from=nvidia-libs \
    /usr/lib/x86_64-linux-gnu/libnvinfer.so.10.13.2 \
    /usr/lib/x86_64-linux-gnu/libnvinfer.so.10 \
    /usr/lib/x86_64-linux-gnu/
RUN ln -sf libnvinfer.so.10.13.2 /usr/lib/x86_64-linux-gnu/libnvinfer.so.10 && \
    ln -sf libnvinfer.so.10       /usr/lib/x86_64-linux-gnu/libnvinfer.so

COPY --from=nvidia-libs \
    /usr/lib/x86_64-linux-gnu/libnvinfer_plugin.so.10.13.2 \
    /usr/lib/x86_64-linux-gnu/libnvinfer_plugin.so.10 \
    /usr/lib/x86_64-linux-gnu/
RUN ln -sf libnvinfer_plugin.so.10.13.2 /usr/lib/x86_64-linux-gnu/libnvinfer_plugin.so.10 && \
    ln -sf libnvinfer_plugin.so.10       /usr/lib/x86_64-linux-gnu/libnvinfer_plugin.so

COPY --from=nvidia-libs \
    /usr/lib/x86_64-linux-gnu/libnvonnxparser.so.10.13.2 \
    /usr/lib/x86_64-linux-gnu/libnvonnxparser.so.10 \
    /usr/lib/x86_64-linux-gnu/
RUN ln -sf libnvonnxparser.so.10.13.2 /usr/lib/x86_64-linux-gnu/libnvonnxparser.so.10 && \
    ln -sf libnvonnxparser.so.10       /usr/lib/x86_64-linux-gnu/libnvonnxparser.so

COPY --from=nvidia-libs \
    /usr/lib/x86_64-linux-gnu/libcudnn.so.9.12.0 \
    /usr/lib/x86_64-linux-gnu/libcudnn.so.9 \
    /usr/lib/x86_64-linux-gnu/libcudnn_adv.so.9.12.0 \
    /usr/lib/x86_64-linux-gnu/libcudnn_cnn.so.9.12.0 \
    /usr/lib/x86_64-linux-gnu/libcudnn_engines_precompiled.so.9.12.0 \
    /usr/lib/x86_64-linux-gnu/libcudnn_engines_runtime_compiled.so.9.12.0 \
    /usr/lib/x86_64-linux-gnu/libcudnn_graph.so.9.12.0 \
    /usr/lib/x86_64-linux-gnu/libcudnn_heuristic.so.9.12.0 \
    /usr/lib/x86_64-linux-gnu/libcudnn_ops.so.9.12.0 \
    /tmp/cudnn/
RUN ln -sf libcudnn.so.9.12.0 /tmp/cudnn/libcudnn.so.9 && \
    ln -sf libcudnn.so.9       /tmp/cudnn/libcudnn.so && \
    ln -sf libcudnn_adv.so.9.12.0 /tmp/cudnn/libcudnn_adv.so.9 && \
    ln -sf libcudnn_adv.so.9       /tmp/cudnn/libcudnn_adv.so && \
    ln -sf libcudnn_cnn.so.9.12.0 /tmp/cudnn/libcudnn_cnn.so.9 && \
    ln -sf libcudnn_cnn.so.9       /tmp/cudnn/libcudnn_cnn.so && \
    ln -sf libcudnn_engines_precompiled.so.9.12.0 /tmp/cudnn/libcudnn_engines_precompiled.so.9 && \
    ln -sf libcudnn_engines_precompiled.so.9       /tmp/cudnn/libcudnn_engines_precompiled.so && \
    ln -sf libcudnn_engines_runtime_compiled.so.9.12.0 /tmp/cudnn/libcudnn_engines_runtime_compiled.so.9 && \
    ln -sf libcudnn_engines_runtime_compiled.so.9       /tmp/cudnn/libcudnn_engines_runtime_compiled.so && \
    ln -sf libcudnn_graph.so.9.12.0 /tmp/cudnn/libcudnn_graph.so.9 && \
    ln -sf libcudnn_graph.so.9       /tmp/cudnn/libcudnn_graph.so && \
    ln -sf libcudnn_heuristic.so.9.12.0 /tmp/cudnn/libcudnn_heuristic.so.9 && \
    ln -sf libcudnn_heuristic.so.9       /tmp/cudnn/libcudnn_heuristic.so && \
    ln -sf libcudnn_ops.so.9.12.0 /tmp/cudnn/libcudnn_ops.so.9 && \
    ln -sf libcudnn_ops.so.9       /tmp/cudnn/libcudnn_ops.so && \
    cp -a /tmp/cudnn/* /usr/lib/x86_64-linux-gnu/ && rm -rf /tmp/cudnn && \
    ldconfig

COPY --from=build /opt/libft4222/libft4222.so.1.4.4.232 /usr/local/lib/
RUN ln -sf libft4222.so.1.4.4.232 /usr/local/lib/libft4222.so.1 && \
    ln -sf libft4222.so.1 /usr/local/lib/libft4222.so && ldconfig

COPY --from=build /opt/laser_guidance /opt/laser_guidance
ENV PATH="/opt/laser_guidance/bin:${PATH}"

RUN mkdir -p /workspace/laser_guidance/config \
             /workspace/laser_guidance/models \
             /workspace/laser_guidance/test_data \
             /workspace/laser_guidance/videos
COPY --from=build /workspace/laser_guidance/config  /workspace/laser_guidance/config
COPY --from=build /workspace/laser_guidance/models  /workspace/laser_guidance/models
COPY --from=build /workspace/laser_guidance/test_data /workspace/laser_guidance/test_data
WORKDIR /workspace/laser_guidance

ENTRYPOINT ["tini", "--"]
CMD ["tool_competition", "config/direct_voltage_run.yaml"]
