FROM osrf/ros:jazzy-desktop-full-noble

SHELL [ "/bin/bash" , "-c" ]

# Remove old ROS key and install updated one
RUN sudo apt-get install -y curl gnupg lsb-release && \
    sudo rm -f /etc/apt/sources.list.d/ros2-latest.list && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    sudo apt-get update

# Now safe to do full upgrade
RUN sudo apt-get upgrade -y

RUN sudo apt update && sudo apt upgrade -y

# Install pycram dependencies
RUN sudo apt install wget python3.12-venv ros-jazzy-xacro python3-vcstool git ros-dev-tools default-jre -y

# Install RDR dependencies
RUN sudo apt-get install graphviz graphviz-dev -y

# Set the user to the non-root user
ARG USERNAME=root

# Switch to non-root user
USER $USERNAME
ENV HOME=/root

# Create overlay workspace
WORKDIR $HOME/workspace/src
COPY packages.repos .

ARG LIBSDIR=$HOME/libs

RUN mkdir -p $LIBSDIR && \
    cd $LIBSDIR && \
    git clone -b giskard_library https://github.com/SemRoCo/giskardpy.git && \
    git clone -b main https://github.com/AbdelrhmanBassiouny/Multiverse-Parser.git && \
    git clone -b main https://github.com/Multiverse-Framework/Multiverse-Resources.git && \
    git clone https://github.com/qpSWIFT/qpSWIFT


ARG BRANCH_NAME=icub_demo_ros2

RUN vcs import --input https://raw.githubusercontent.com/AbdelrhmanBassiouny/pycram/${BRANCH_NAME}/rosinstall/pycram-ros2.rosinstall \
    && cd .. \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --symlink-install

RUN vcs import --input ./packages.repos --recursive
RUN cd $HOME/workspace/src/ripple_down_rules && \
    git pull

# Add sourcing ROS setup.bash to .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> $HOME/.bashrc

# Install virtualenv and virtualenvwrapper
RUN python3 -m venv $HOME/.virtualenvs/venv && \
    source $HOME/.virtualenvs/venv/bin/activate && \
    pip install --upgrade pip && \
    pip install virtualenvwrapper

# Add virtualenvwrapper to bashrc
RUN echo "export WORKON_HOME=$HOME/.virtualenvs" >> $HOME/.bashrc && \
    echo "export VIRTUALENVWRAPPER_PYTHON=$HOME/.virtualenvs/venv/bin/python" >> $HOME/.bashrc && \
    echo "export VIRTUALENVWRAPPER_VIRTUALENV=$HOME/.virtualenvs/venv/bin/virtualenv" >> $HOME/.bashrc && \
    echo "source $HOME/.virtualenvs/venv/bin/virtualenvwrapper.sh" >> $HOME/.bashrc \

RUN echo "export PYTHONPATH=\$PYTHONPATH:$HOME/workspace/src/Multiverse-Parser/USD/linux" >> $HOME/.bashrc


ENV WORKON_HOME=$HOME/.virtualenvs
ENV VIRTUALENVWRAPPER_PYTHON=$HOME/.virtualenvs/venv/bin/python
ENV VIRTUALENVWRAPPER_VIRTUALENV=$HOME/.virtualenvs/venv/bin/virtualenv

# Create a virtual environment for pycram-segmind
RUN source $HOME/.virtualenvs/venv/bin/virtualenvwrapper.sh && \
    mkvirtualenv --system-site-packages pycram-segmind && \
    workon pycram-segmind && \
    pip install --upgrade pip && \
    pip install -r pycram/requirements.txt && \
    pip install -U setuptools && \
    pip install -r ripple_down_rules/requirements.txt && \
    pip install lark && \
    cd $LIBSDIR/giskardpy && \
    pip install -r requirements.txt &&  pip install -e . && \
    cd $LIBSDIR/Multiverse-Parser && \
    pip install -r requirements.txt && pip install -e . && \
    cd $LIBSDIR/qpSWIFT/python && \
    sudo python3 setup.py install && \
    && source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd $HOME/workspace && \
    vcs import src < src/giskardpy_ros/$ROS_DISTRO.repos && \
    rosdep update --rosdistro=$ROS_DISTRO && \
    sudo apt-get update && \
    rosdep install --from-paths ./ -i -y --rosdistro ${ROS_DISTRO} && \
    sudo apt-get install ros-jazzy-rclpy-message-converter -y && \
    colcon build --symlink-install && \
    source $HOME/workspace/install/setup.bash

# Install code-server
RUN sudo apt-get update && sudo apt-get install -y \
    curl wget gnupg git bash sudo build-essential \
    && sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
RUN curl -fsSL https://code-server.dev/install.sh | sh
# Install desired extensions
# Set code-server user data dir manually if your container does this
#ENV USER_DATA_DIR=/.jbdevcontainer/data/code-server
ENV USER_DATA_DIR=$HOME/.local/share/code-server
# Create the data dir
RUN mkdir -p $USER_DATA_DIR
# Install extensions into the proper location
RUN code-server \
    --user-data-dir=$USER_DATA_DIR \
    --install-extension ms-python.python \
    && code-server \
    --user-data-dir=$USER_DATA_DIR \
    --install-extension Codeium.codeium \
    && code-server \
    --user-data-dir=$USER_DATA_DIR \
    --install-extension PKief.material-icon-theme \
    && code-server \
    --user-data-dir=$USER_DATA_DIR \
    --install-extension virgilsisoe.python-auto-import

COPY start-code-server.sh /usr/local/bin/start-code-server.sh
RUN sudo chmod +x /usr/local/bin/start-code-server.sh

COPY post_create_setup.sh /
RUN sudo chmod +x /post_create_setup.sh

ENV XDG_RUNTIME_DIR=/tmp/runtime-root
RUN mkdir -p $XDG_RUNTIME_DIR && chmod 700 $XDG_RUNTIME_DIR

ENV LIBGL_ALWAYS_SOFTWARE=1

RUN echo "source ${HOME}/.virtualenvs/pycram-segmind/bin/activate" >> $HOME/.bashrc
RUN echo "source ${HOME}/workspace/install/setup.bash" >> $HOME/.bashrc

COPY entrypoint.sh /
RUN sudo chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
