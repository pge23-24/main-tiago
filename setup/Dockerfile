FROM palroboticssl/pal_mobile_base_tutorials:noetic

USER root

# Install network-manager
RUN apt update && apt install -y network-manager

# QT config
RUN echo "export QT_X11_NO_MITSHM=1" >> ~/.bashrc

RUN mkdir -m 700 -p /run/user/$(id -u)
RUN echo "export XDG_RUNTIME_DIR=/run/user/$(id -u)" >> ~/.bashrc

# Add shortcuts and aliases
COPY aliases.bash /root/.bash_aliases