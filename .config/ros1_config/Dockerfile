FROM palroboticssl/pal_mobile_base_tutorials:noetic

USER root

# Install network-manager
RUN apt update && apt install -y network-manager tmux

# Install pal urdf utils
RUN cd /pal_mobile_base_ws/src && \
    git clone https://github.com/pal-robotics/pal_urdf_utils.git && \
    cd pal_urdf_utils && \
    catkin build --this

# Remove unused packages
RUN rm -rf /pal_mobile_base_ws/src/pmb3*

# QT config
RUN echo "export QT_X11_NO_MITSHM=1" >> ~/.bashrc

RUN mkdir -m 700 -p /run/user/$(id -u)
RUN echo "export XDG_RUNTIME_DIR=/run/user/$(id -u)" >> ~/.bashrc

# Add ssh fingerprint for robot
RUN mkdir -p /root/.ssh/ && printf '%s\n' \
    '|1|vrH/WCM7BuiEluWWTl3deeLVe+Q=|ZDvqX33v9VvG7/aAj1z8lIWhxWc= ecdsa-sha2-nistp256 AAAAE2VjZHNhLXNoYTItbmlzdHAyNTYAAAAIbmlzdHAyNTYAAABBBA8ZIP0+Ka64PYbmGihatETM/lTHTtNZAH1o5ZPi0qiaLKA7uS0drlHScPRjdNSldA93NrQNM0rW8h8aNMGpaxU=' \
    '|1|+FEkZ7sLE2MpvF1uIX5SU0bQMQI=|fB04/hTrqxzuZAoUUDK1ZUBOn+I= ecdsa-sha2-nistp256 AAAAE2VjZHNhLXNoYTItbmlzdHAyNTYAAAAIbmlzdHAyNTYAAABBBA8ZIP0+Ka64PYbmGihatETM/lTHTtNZAH1o5ZPi0qiaLKA7uS0drlHScPRjdNSldA93NrQNM0rW8h8aNMGpaxU=' \
    > /root/.ssh/known_hosts


# Add shortcuts and aliases
COPY aliases.bash /root/.bash_aliases
