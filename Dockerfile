FROM osrf/ros:humble-desktop


# Неинтерактивный режим для apt
ENV DEBIAN_FRONTEND=noninteractive



# Аргументы для UID/GID — передаём при сборке, по умолчанию 1000
ARG USERNAME=rosdev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Удаляем пользователя, если он уже есть с таким UID (иногда в базовом образе есть ubuntu или другой)
RUN if id -u $USER_UID > /dev/null 2>&1; then userdel -f $(id -un $USER_UID); fi

# Создаём группу и пользователя с нужными UID/GID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # Даём возможность использовать sudo без пароля (удобно для apt внутри контейнера)
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    # Чистим кэш
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean


# Обновляем и устанавливаем Gazebo Fortress + ROS-GZ bridge + urdf_tutorial
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ignition-fortress \
        ros-humble-ros-gz \
        ros-humble-ros-gz-bridge \
        ros-humble-ros-gz-sim \
        ros-humble-urdf-tutorial \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean


# Переключаемся на созданного пользователя
USER $USERNAME

# Устанавливаем переменные для GUI (X11)
ENV QT_X11_NO_MITSHM=1
ENV DISPLAY=:0

# Автозагрузка ROS в bash
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

# Рабочая директория — твоя src будет монтироваться сюда
WORKDIR /home/ros_ws/src

# По умолчанию запускаем bash
CMD ["bash"]






