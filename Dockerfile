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
    libopencv-dev libyaml-cpp-dev libzmq3-dev cppzmq-dev \
    libusb-1.0-0-dev \
    libavcodec-dev libavformat-dev libavutil-dev libswscale-dev \
    ffmpeg \
    python3 python3-yaml \
    ca-certificates curl wget \
    v4l-utils usbutils \
    software-properties-common \
  && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-14 50 \
  && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-14 50 \
  && apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/*

# ROS2 Jazzy (build deps)
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list \
  && apt-get update \
  && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-base ros-jazzy-rclcpp \
    ros-jazzy-visualization-msgs ros-jazzy-std-msgs \
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
    . /opt/ros/jazzy/setup.sh && \
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
    software-properties-common curl \
  && apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/*

# ROS2 Jazzy runtime + Foxglove bridge
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list \
  && apt-get update \
  && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-base ros-jazzy-foxglove-bridge \
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

# Startup script: source ROS2, launch foxglove bridge, run tool
RUN printf '#!/bin/bash\n\
set -e\n\
source /opt/ros/jazzy/setup.bash\n\
echo "[entry] Starting foxglove_bridge on port 8765..."\n\
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 &\n\
FOXGLOVE_PID=$!\n\
sleep 2\n\
echo "[entry] Starting laser_guidance..."\n\
"$@"\n\
LASER_EXIT=$?\n\
kill $FOXGLOVE_PID 2>/dev/null || true\n\
wait $FOXGLOVE_PID 2>/dev/null || true\n\
exit $LASER_EXIT\n' > /opt/laser_guidance/bin/entrypoint.sh \
  && chmod +x /opt/laser_guidance/bin/entrypoint.sh

ENTRYPOINT ["tini", "--"]
CMD ["/opt/laser_guidance/bin/entrypoint.sh", "tool_competition", "config/direct_voltage_run.yaml"]

# ---- Develop stage (build tools + IDE/agent support) --------------------------
FROM runtime AS develop

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build pkg-config gdb lldb git \
    libopencv-dev libyaml-cpp-dev \
    sudo zsh wget curl ca-certificates gnupg \
    software-properties-common lsb-release \
  && apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/*

# Node.js 24 (for opencode/codex/claude CLI tools)
RUN curl -fsSL https://deb.nodesource.com/setup_24.x | bash - \
  && apt-get install -y --no-install-recommends nodejs \
  && apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/*

# LLVM toolchain (clangd + clang-format + clang-tidy)
RUN wget -qO- https://apt.llvm.org/llvm-snapshot.gpg.key | gpg --dearmor -o /etc/apt/keyrings/apt.llvm.org.gpg \
  && chmod 644 /etc/apt/keyrings/apt.llvm.org.gpg \
  && echo "deb [signed-by=/etc/apt/keyrings/apt.llvm.org.gpg] https://apt.llvm.org/noble/ llvm-toolchain-noble-19 main" \
    > /etc/apt/sources.list.d/llvm.list \
  && apt-get update \
  && apt-get install -y --no-install-recommends \
    clang-19 clangd-19 clang-format-19 clang-tidy-19 \
    lld-19 llvm-19 \
  && update-alternatives --install /usr/bin/clang clang /usr/bin/clang-19 50 \
  && update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-19 50 \
  && update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-19 50 \
  && update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-19 50 \
  && update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-19 50 \
  && apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/*

# neovim (latest prebuilt release)
RUN curl -LO https://github.com/neovim/neovim/releases/latest/download/nvim-linux-x86_64.tar.gz \
  && rm -rf /opt/nvim \
  && tar -C /opt -xzf nvim-linux-x86_64.tar.gz \
  && mv /opt/nvim-linux-x86_64 /opt/nvim \
  && rm nvim-linux-x86_64.tar.gz
ENV PATH="/opt/nvim/bin:${PATH}"

# ONNX Runtime dev headers (for local cmake development)
COPY --from=deps /opt/onnxruntime/include /opt/onnxruntime/include
COPY --from=deps /opt/onnxruntime/lib/cmake /opt/onnxruntime/lib/cmake
ENV ONNXRUNTIME_ROOT=/opt/onnxruntime

# CUDA headers (for local TensorRT dev)
COPY --from=build /usr/local/cuda/include /usr/local/cuda/include

# yukikaze user (UID 1000, matches host)
RUN useradd -m -u 1000 -o -s /bin/zsh yukikaze \
  && echo "yukikaze ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers \
  && mkdir -p /home/yukikaze/.config /home/yukikaze/.local/share /home/yukikaze/.agents \
  && chown -R yukikaze:yukikaze /home/yukikaze

# oh-my-zsh for yukikaze
USER yukikaze
RUN sh -c "$(wget -qO- https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended \
  && sed -i 's/ZSH_THEME="[a-z0-9\-]*"/ZSH_THEME="af-magic"/g' ~/.zshrc \
  && echo 'source /opt/ros/jazzy/setup.zsh' >> ~/.zshrc

# Root fallback: oh-my-zsh for root too
USER root
RUN sh -c "$(wget -qO- https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended \
  && sed -i 's/ZSH_THEME="[a-z0-9\-]*"/ZSH_THEME="af-magic"/g' ~/.zshrc \
  && echo 'source /opt/ros/jazzy/setup.zsh' >> ~/.zshrc

WORKDIR /workspace/laser_guidance
CMD ["/bin/zsh"]
