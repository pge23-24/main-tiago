FROM palroboticssl/pal_mobile_base_tutorials:noetic

USER root

# Move workspace
# RUN ln -s /pal_mobile_base_ws /workspace
# RUN sed -i 's/pal_mobile_base_ws/workspace/g' ~/.bashrc
# WORKDIR /workspace

# Install network-manager
RUN apt update && apt install -y network-manager tmux

# Install pal urdf utils
RUN cd /pal_mobile_base_ws/src && \
    git clone https://github.com/pal-robotics/pal_urdf_utils.git && \
    cd pal_urdf_utils && \
    catkin build --this

# QT config
RUN echo "export QT_X11_NO_MITSHM=1" >> ~/.bashrc

RUN mkdir -m 700 -p /run/user/$(id -u)
RUN echo "export XDG_RUNTIME_DIR=/run/user/$(id -u)" >> ~/.bashrc

# Add shortcuts and aliases
# RUN echo "cd /workspace" >> /root/.bashrc
COPY aliases.bash /root/.bash_aliases
