FROM ros:noetic-ros-core

# タイムゾーンを JST に設定
ENV TZ=Asia/Tokyo
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime \
 && echo $TZ > /etc/timezone

# 必要な apt パッケージのインストール
RUN apt-get update && apt-get install -y \
      git \
      python3-pip \
      python3-catkin-tools \
      python3-rosdep \
      qtbase5-dev \
      qtbase5-dev-tools \
      libqt5core5a \
      libqt5gui5 \
      libqt5widgets5 \
      libhdf5-dev \
      ros-noetic-actionlib\
      ros-noetic-tf\
      ros-noetic-cv-bridge\
      python3-opencv\
      libopencv-dev\
      ros-noetic-moveit-ros-planning-interface\
      ros-noetic-control-msgs\
      libyaml-cpp-dev\
      pkg-config\
      ros-noetic-rviz\
      ros-noetic-sparse-bundle-adjustment\
      ros-noetic-map-server\
      libceres-dev\
      ros-noetic-move-base\
      ros-noetic-interactive-markers\
      libceres1\
      ros-noetic-teb-local-planner\
      dos2unix \

    && rm -rf /var/lib/apt/lists/*

# rosdep 初期化
RUN rosdep init \
 && rosdep update


# 作業ディレクトリ設定
WORKDIR /root

# デフォルトシェル
CMD ["bash"]
