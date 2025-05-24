FROM osrf/ros:jazzy-desktop-full

SHELL [ "/bin/bash" , "-c" ]

# Upgrade all packages
RUN sudo apt update && sudo apt upgrade -y

# Install pycram dependencies
RUN sudo apt install wget python3.12-venv ros-jazzy-xacro python3-vcstool git ros-dev-tools default-jre -y

# Install RDR dependencies
RUN sudo apt-get install graphviz graphviz-dev -y

# Create overlay workspace
WORKDIR /root/ros_ws/src
COPY packages.repos .


RUN vcs import --input https://raw.githubusercontent.com/AbdelrhmanBassiouny/pycram/dev/pycram-ros2.rosinstall \
    && cd .. \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --symlink-install

RUN vcs import --input packages.repos --recursive

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
    echo "source $HOME/.virtualenvs/venv/bin/virtualenvwrapper.sh" >> $HOME/.bashrc


ENV WORKON_HOME=/root/.virtualenvs
ENV VIRTUALENVWRAPPER_PYTHON=/root/.virtualenvs/venv/bin/python
ENV VIRTUALENVWRAPPER_VIRTUALENV=/root/.virtualenvs/venv/bin/virtualenv

# Create a virtual environment for pycram-segmind
RUN source $HOME/.virtualenvs/venv/bin/virtualenvwrapper.sh && \
    mkvirtualenv --system-site-packages pycram-segmind && \
    workon pycram-segmind && \
    pip install --upgrade pip && \
    pip install -r pycram/requirements.txt && \
    pip install -U setuptools && \
    pip install -e ripple_down_rules && \
    pip install lark

COPY post_create_setup.sh /
RUN chmod +x /post_create_setup.sh

COPY test_pick_up.sh /
RUN chmod +x /test_pick_up.sh

RUN echo "source /root/.virtualenvs/pycram-segmind/bin/activate" >> $HOME/.bashrc
RUN echo "source /root/ros_ws/install/setup.bash" >> $HOME/.bashrc

COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
