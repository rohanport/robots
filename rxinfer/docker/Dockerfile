########################
#  RxInfer Environment #
########################
FROM julia:latest AS rxinfer
SHELL ["/bin/bash", "-c"]

# Install basic apt packages
RUN apt-get update && apt-get install -y --no-install-recommends \
 git sudo curl

# Default powerline10k theme, no plugins installed
RUN sh -c "$(curl -L https://github.com/deluan/zsh-in-docker/releases/download/v1.2.0/zsh-in-docker.sh)"

# Setup the rxinfer julia environment
RUN mkdir /workspace
RUN mkdir /workspace/rxinfer
WORKDIR /workspace/rxinfer
COPY ./rxinfer/ ./

ARG USERNAME=rxinfer
ARG UID=1000
ARG GID=${UID}

# Create new user and home directory
RUN groupadd --gid $GID $USERNAME \
 && useradd --uid ${GID} --gid ${UID} --create-home ${USERNAME} \
 && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && mkdir -p /home/${USERNAME} \
 && chown -R ${UID}:${GID} /home/${USERNAME}

# Set the ownership of the overlay workspace to the new user
RUN chown -R ${UID}:${GID} /workspace/rxinfer/

# Set the user and source entrypoint in the user's .bashrc file
COPY ./rxinfer/docker/entrypoint.sh /
RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc
USER ${USERNAME}

# Permanently set the JULIA_PROJECT
ENV JULIA_PROJECT "/workspace/rxinfer"

# Install project dependencies
RUN julia -e 'using Pkg; Pkg.instantiate()'