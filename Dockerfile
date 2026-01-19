FROM ros:noetic-ros-core

# タイムゾーンを JST に設定
ENV TZ=Asia/Tokyo
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime \
 && echo $TZ > /etc/timezone

# 必要な apt パッケージのインストール
RUN apt-get update && apt-get install -y --no-install-recommends \
      git \
      python3-pip \
      python3-catkin-tools \
      python3-rosdep \
      libnss-wrapper \
      qtbase5-dev \
      qtbase5-dev-tools \
      libqt5core5a \
      libqt5gui5 \
      libqt5widgets5 \
      libhdf5-dev \
      ros-noetic-actionlib \
      ros-noetic-tf \
      ros-noetic-cv-bridge \
      python3-opencv \
      libopencv-dev \
      ros-noetic-moveit-ros-planning-interface \
      ros-noetic-control-msgs \
      libyaml-cpp-dev \
      build-essential \
      cmake \
      pkg-config \
      ros-noetic-rviz \
      ros-noetic-sparse-bundle-adjustment \
      ros-noetic-map-server \
      libceres-dev \
      ros-noetic-move-base \
      ros-noetic-interactive-markers \
      libceres1 \
      ros-noetic-teb-local-planner \
      dos2unix \
      ros-noetic-xacro \
      ros-noetic-robot-state-publisher \
      ros-noetic-kdl-parser-py \
      python3-pykdl \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip \
 && pip3 install --no-cache-dir open3d \
 && pip3 install --no-cache-dir "numpy==1.24.4" --upgrade --ignore-installed


# ------- ここまで -------

# rosdep 初期化
RUN rosdep init \
 && rosdep update


# ★追加：新entrypointを配置
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# ★重要：/work をデフォルト作業場所にする（非root運用と整合）
WORKDIR /work

# ★変更：/ros_entrypoint.sh をラップする
ENTRYPOINT ["/entrypoint.sh"]

# デフォルトシェル
CMD ["bash"]
