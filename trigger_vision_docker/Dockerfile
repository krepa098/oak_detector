FROM nvidia/cuda:12.1.1-cudnn8-devel-ubuntu22.04 as base
# https://github.com/microsoft/wslg/blob/main/samples/container/Containers.md

ARG ROS_WS_NAME=ros_ws
ARG ROS2_DISTRO=humble

ARG USERNAME=sntarg
ARG PASSWD=pwd

ARG DEBIAN_FRONTEND=noninteractive

ENV TZ=Europe/Rome

ENV LIBVA_DRIVER_NAME=d3d12
ENV LD_LIBRARY_PATH ${LD_LIBRARY_PATH:+${LD_LIBRARY_PATH}:}/usr/lib/wsl/lib
#https://github.com/NVIDIA/nvidia-docker/issues/1438
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,video

RUN apt-get update && apt-get upgrade -y &&\
    apt-get install -y \
    tzdata sudo \
    build-essential \
    software-properties-common \
    xdg-user-dirs \
    pkg-config \
    gcc \
    g++ \
    gdb \
    clang \
    cmake \
    pkg-config

RUN add-apt-repository ppa:oibaf/graphics-drivers -y

RUN apt update && apt install -y \
    vainfo \
    mesa-va-drivers \
    mesa-utils



RUN apt-get update && apt-get upgrade -y &&\
    apt-get install -y \
    libdw-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev libtiff5-dev \
    libavformat-dev \
    libpq-dev \
    libxine2-dev \
    libglew-dev \
    zlib1g-dev \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libpostproc-dev \
    libswscale-dev \
    libeigen3-dev \
    libtbb2 libtbb-dev \
    libssl-dev \
    libffi-dev \
    libgtk2.0-dev \
    libqt5opengl5-dev \
    qtbase5-dev qt5-qmake \
    libfontconfig1-dev \
    libfreetype6-dev \
    libx11-dev \
    libx11-xcb-dev \
    libxext-dev \
    libxfixes-dev \
    libxi-dev \
    libxrender-dev \
    libxcb1-dev \
    libxcb-glx0-dev \
    libxcb-keysyms1-dev \
    libxcb-image0-dev \
    libxcb-shm0-dev \
    libxcb-icccm4-dev \
    libxcb-sync0-dev \
    libxcb-xfixes0-dev \
    libxcb-shape0-dev \
    libxcb-randr0-dev \
    libxcb-render-util0-dev \
    libxkbcommon-dev \
    libxkbcommon-x11-dev \
    libxinerama-dev

RUN apt-get install -y \
    python3-dev \
    python3-pip \
    python3-venv \
    python3-tk \
    tmux \
    iputils-ping \
    iproute2 \
    net-tools \
    bash-completion  \
    htop \
    usbutils \
    ssh \
    rsync \
    tar \
    git \
    wget \
    curl \
    unzip \
    yasm \
    vim \
    nano

#ROS2 Installation
RUN locale
RUN apt update && sudo apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
RUN locale

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update && apt install -y ros-${ROS2_DISTRO}-desktop python3-argcomplete ros-dev-tools python3-colcon-common-extensions ros-${ROS2_DISTRO}-depthai-ros\
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init

# Local user setup
ENV USERNAME ${USERNAME}
ENV PASSWD ${PASSWD}
ENV SHELL /bin/bash
ENV HOME /home/${USERNAME}
ENV ROS_WS ${HOME}/${ROS_WS_NAME}
ENV PATH $HOME/bin/:$HOME/.local/bin${PATH:+:${PATH}}
ENV QT_X11_NO_MITSHM=1

# https://serverfault.com/questions/98842/useradd-with-755-permissions
RUN useradd -m -u 1000 -U -p ${PASSWD} -s ${SHELL} -G video -K UMASK=0022 ${USERNAME}

RUN usermod -a -G sudo ${USERNAME}  && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

RUN usermod --password $(openssl passwd -1 ${PASSWD}) ${USERNAME}

RUN usermod --password $(openssl passwd -1 ${PASSWD}) root

#https://stackoverflow.com/questions/34630571/docker-env-variables-not-set-while-log-via-shell
RUN env | grep _ >> /etc/environment
RUN mkdir /root/.ssh
RUN env | grep _ >> /root/.ssh/environment

RUN SNIPPET="export PROMPT_COMMAND='history -a' && export HISTFILE=/bashhistory/.bash_history" \
    && mkdir /bashhistory \
    && touch /bashhistory/.bash_history \
    && chown -R $USERNAME /bashhistory \
    && echo "$SNIPPET" >> "/home/$USERNAME/.bashrc"

RUN echo "Dir::Cache::srcpkgcache "";" > etc/apt/apt.conf.d/docker-clean

USER $USERNAME

# https://github.com/microsoft/WSL/issues/7507
# ENV LIBVA_DRIVER_NAME=d3d12
# ENV LD_LIBRARY_PATH ${LD_LIBRARY_PATH:+${LD_LIBRARY_PATH}:}/usr/lib/wsl/lib
# https://github.com/microsoft/wslg/wiki/GPU-selection-in-WSLg
# ENV MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
ENV LIBGL_ALWAYS_SOFTWARE=1

RUN xdg-user-dirs-update --force

# RUN mkdir -p /tmp/runtime-ros
# RUN echo 'export XDG_RUNTIME_DIR=/tmp/runtime-ros' >> ~/.bashrc

RUN mkdir ${HOME}/.ssh

RUN env | grep _ >> ${HOME}/.ssh/environment

RUN mkdir -p $ROS_WS/src

RUN rosdep update
ENV ROS2_DISTRO=${ROS2_DISTRO}
RUN echo "test -e /opt/ros/${ROS2_DISTRO}/setup.bash && source /opt/ros/${ROS2_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "test -e ${ROS_WS}/install/local_setup.bash && source ${ROS_WS}/install/local_setup.bash" >> ~/.bashrc
RUN echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
RUN echo "export ROS_DOMAIN_ID=3" >> ~/.bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/${ROS2_DISTRO}/" >> ~/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

RUN echo "alias ..='cd ..'" >>  ~/.bash_aliases

ENV CONDA_ROOT=${HOME}/miniconda3
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh
RUN bash ~/miniconda.sh -b -u -p ${CONDA_ROOT}
RUN rm -rf ~/miniconda.sh
ENV CONDA_INIT_SKIP=true

SHELL ["/bin/bash", "-c"]
ENV PATH=${CONDA_ROOT}/bin:${PATH}

RUN pip install -U colcon-common-extensions

RUN conda init bash && conda update conda && conda install -c conda-forge conda-bash-completion && conda create --name ros -c conda-forge python=3.11

RUN conda config --set auto_activate_base false

RUN source $HOME/miniconda3/etc/profile.d/conda.sh && \
    conda activate ros &&\
    pip install Cython numpy scipy \
    matplotlib tqdm pillow \
    pandas pyyaml opencv-python autopep8

WORKDIR ${HOME}

RUN source $HOME/miniconda3/etc/profile.d/conda.sh && \
    conda activate ros &&\ wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash && \
    python3 -m pip install --extra-index-url https://artifacts.luxonis.com/artifactory/luxonis-python-snapshot-local/ depthai && \
    git clone https://github.com/luxonis/depthai.git && \
    cd depthai/ && \
    python3 install_requirements.py
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules


RUN printf "if [[ -r $CONDA_ROOT/etc/profile.d/bash_completion.sh ]]; then\n\
    source $CONDA_ROOT/etc/profile.d/bash_completion.sh\n\
else\n\
    echo 'WARNING: could not find conda-bash-completion setup script'\n\
fi\n" >> ~/.bashrc

RUN printf "conda activate ros" >> ~/.bashrc


# RUN pip3 install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu117

# RUN pip3 install transformers timm open3d

# RUN sudo sed -i ''$(($(wc -l < /usr/local/bin/nvidia_entrypoint.sh)-4))'i rosdep update && sudo apt update && rosdep install --from-paths . -y --ignore-src \n sleep 3 \n' /usr/local/bin/nvidia_entrypoint.sh
