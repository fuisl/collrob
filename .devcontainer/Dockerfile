FROM ros:humble

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

ENV SHELL=/bin/zsh

# Base packages
RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y sudo curl ca-certificates git zsh \
    && rm -rf /var/lib/apt/lists/*

# Create dev user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/zsh $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install uv system-wide
ENV UV_INSTALL_DIR=/usr/local/bin
RUN curl -LsSf https://astral.sh/uv/install.sh | sh

RUN apt-get update && apt-get install -y \
    ros-humble-ros-gz \
    ros-humble-ros-ign-bridge \
    ros-humble-rviz2 \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    && apt-get clean && rm -rf /var/lib/apt/lists/*
# Install oh-my-zsh as dev user
USER $USERNAME
WORKDIR /home/$USERNAME

RUN RUNZSH=no CHSH=no KEEP_ZSHRC=yes \
    sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"

# ROS in zsh + minimal plugins/theme
RUN echo 'source /opt/ros/humble/setup.zsh' >> ~/.zshrc \
    && echo 'export ZSH_THEME="robbyrussell"' >> ~/.zshrc \
    && echo 'plugins=(git)' >> ~/.zshrc

CMD ["/bin/zsh"]