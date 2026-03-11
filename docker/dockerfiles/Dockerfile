# check=skip=InvalidDefaultArgInFrom
ARG SOURCE_IMAGE
FROM ${SOURCE_IMAGE}

ARG USER_NAME
ARG USER_ID
ARG GROUP_ID
ARG USER_HOME
ARG PROJECT_NAME

RUN mkdir -p ${USER_HOME} && \
    echo "${USER_NAME}:x:${USER_ID}:${GROUP_ID}::${USER_HOME}:/bin/bash" >> /etc/passwd && \
    echo "${USER_NAME}:x:${GROUP_ID}:" >> /etc/group && \
    mkdir -p /etc/sudoers.d && \
    echo "${USER_NAME} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/${USER_NAME} && \
    chmod 0440 /etc/sudoers.d/${USER_NAME} && \
    chown ${USER_ID}:${GROUP_ID} -R ${USER_HOME} && \
    echo "${USER_NAME}:${USER_NAME}" | chpasswd && \
    echo "root:root" | chpasswd

COPY entrypoint.sh /
COPY .bashrc ${USER_HOME}

USER ${USER_NAME}
ENV HOME=${USER_HOME}

ENTRYPOINT ["/entrypoint.sh"]
