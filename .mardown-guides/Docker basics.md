# Basics of Docker

## Table of Contents

- [Basics of Docker](#basics-of-docker)
  - [Table of Contents](#table-of-contents)
  - [Test if Docker is installed](#test-if-docker-is-installed)
  - [Dockerfile](#dockerfile)
    - [FROM](#from)
    - [LABEL](#label)
    - [RUN](#run)
    - [COPY](#copy)
    - [ARGS](#args)
    - [ENV](#env)
    - [WORKDIR](#workdir)
    - [ENTRYPOINT](#entrypoint)
    - [CMD](#cmd)
    - [EXPOSE](#expose)
  - [Different terminal commands](#different-terminal-commands)
    - [`images`](#images)
    - [`pull <name>:<tag>`](#pull-nametag)
    - [`build <path>`](#build-path)
      - [`build` optional parameters](#build-optional-parameters)
    - [`rmi <image_name>`](#rmi-image_name)
    - [`run <container_name>:<tag>`](#run-container_nametag)
      - [`run` optional parameters](#run-optional-parameters)
    - [`ps`](#ps)
      - [`ps` Optional parameters](#ps-optional-parameters)
    - [`start <container_name>`](#start-container_name)
    - [`stop <container_name>`](#stop-container_name)
    - [`exec <container_name> <excutable_command>`](#exec-container_name-excutable_command)
    - [`rm <container_name>`](#rm-container_name)
    - [`container prune`](#container-prune)

## Test if Docker is installed

To test if Docker is installed on your machine, run the following command in the terminal:

```bash
docker run hello-world
```

(This will automatically install the `hello-world` image, if not installed)

## Dockerfile

A `Dockerfile` is a file that contains the instructions to build a Docker image

### FROM

Each file should always start with the `FROM` command, which specifies the base image to use.

```Dockerfile
FROM <image>:<tag>
```

The `image` is the name of the image to use, and the `tag` is the version of the image to use.

```Dockerfile
FROM ros:humble
```

### LABEL

The `LABEL` command is used to add metadata to the image.

```Dockerfile
LABEL <key>=<value>
```

The `key` is the name of the metadata, and the `value` is the value of the metadata.

```Dockerfile
LABEL maintainer="John Doe"\
      name="test_image"\
      version="1.0"\
      description="This is a test image"
```

You can add multiple labels by separating them with a `\` and adding a new line.

### RUN

The `RUN` command is used to run commands in the container.

```Dockerfile
RUN <command>
```

The command can be anything that you would run in the terminal.

```Dockerfile
RUN apt-get update \
    && apt-get install -y \
    python3 \
    python3-pip \
    nano \
    && rm -rf /var/lib/apt/lists/*
```

It is a good idea to chain commands together with `&&` to reduce the number of layers in the image. For ease of reading, you can also use `\` to split the command over multiple lines.

When installing using the `apt-get` command, it is a good idea to run `apt-get update` before installing anything, to ensure that the package list is up to date.

Adding the `-y` for the `apt-get install` command will automatically answer `yes` to any prompts, making the installation non-interactive.

Adding `rm -rf /var/lib/apt/lists/*` will remove the package list after installing the packages, to reduce the size of the image.\
This is not necessary, but it is **highly recommended**!

### COPY

The `COPY` command is used to copy files from the host machine to the container.

```Dockerfile
COPY <source> <destination>
```

The `source` is the file or directory on the host machine, and the `destination` is the location in the container.

```Dockerfile
COPY . /app
```

This will copy all of the files in the current directory to the `/app` directory in the container.

### ARGS

The `ARG` command is used to define arguments that can be passed to the `Dockerfile` when building the image.\

```Dockerfile
ARG {name}={default_value}
```

The `name` is the name of the argument, and the `default_value` is the default value to use if the argument is not passed.

```Dockerfile
ARG USER=jetson
```

This will define an argument `USER` with the default value of `jetson`.\
It is not required, but it is recommended to have the `ARG` argument in all caps.
This makes it more distinguishable from other commands.

When running the `build` command, you can pass the argument with the `--build-arg` flag.

```bash
docker build --build-arg USER=ubuntu .
```

To use the argument in the `Dockerfile`, you add `$` before the argument name.

```Dockerfile
RUN echo "User is $USER"
```

This will build the image with the `USER` argument set to `ubuntu`.

### ENV

The `ENV` command is used to set environment variables in the container.\
This is similar to the `ARG` command, but the environment variable is set in the container, not the `Dockerfile`.

```Dockerfile
ENV <name>=<value>
```

The `name` is the name of the environment variable, and the `value` is the value of the environment variable.

```Dockerfile
ENV USER=jetson
```

This will set the environment variable `USER` to `jetson` in the container.

### WORKDIR

The `WORKDIR` command is used to set the working directory in the container.

```Dockerfile
WORKDIR <path>
```

The `path` is the path to the working directory in the container.

```Dockerfile
WORKDIR /app
```

This will set the working directory to `/app` in the container.

### ENTRYPOINT

The `ENTRYPOINT` command is used to specify the command to run when the container starts.

```Dockerfile
ENTRYPOINT <command>
```

The `command` is the command to run when the container starts.

```Dockerfile
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
```

### CMD

The `CMD` command is used to specify the command to run when the container starts.

```Dockerfile
CMD <command>
```

The `command` is the command to run when the container starts.

```Dockerfile
CMD ["python3", "app.py"]
```

This will run the `app.py` file when the container starts.

The `CMD` command can be overridden when running the container by adding the command to the end of the `docker run` command.

```bash
docker run test_image python3 app.py
```

This will run the `app.py` file when the container starts, overriding the `CMD` command in the `Dockerfile`.

### EXPOSE

The `EXPOSE` command is used to specify the port to expose from the container.

```Dockerfile
EXPOSE <port>
```

The `port` is the port to expose from the container.

```Dockerfile
EXPOSE 8080
```

This will expose port `8080` from the container.

## Different terminal commands

### `images`

- `image ls`

Print all of the installed images

### `pull <name>:<tag>`

- `image pull`

Will install the image from [Docker Hub](https://hub.docker.com/).\
If no tag is specified, it will default to `latest`.

### `build <path>`

- `image build <path>`

You can build an image from a `Dockerfile` using the `build` command.\
The `path` is the path to the directory containing the `Dockerfile`.\
If it's in the current directory, you can use `.`

#### `build` optional parameters

- `-t <name>:<tag>`
  - Tags the image with the specified name and tag
  - You can also leave the tag off, and it will default to `latest`

### `rmi <image_name>`

- `image rm`

Removes the specified image\
Adding `-f` will force delete it.

### `run <container_name>:<tag>`

- `container run`

Runs the specified container

#### `run` optional parameters

- `-i`
  - Interactive mode (STDIN)
- `-t`
  - TTY
  - Gives a terminal
- `-it`
  - Combines the `-i` and `-t` parameters
- `--rm`
  - Removes the container after exiting
- `--name <custom_name_for_container>`
  - Names the container
- `--user <...>`
  - Name the user to run the container as
  - Valid combinations:
    - `<user_name>`
    - `<user_name>:<group_name>`
    - `<user_id>`
    - `<user_id>:<group_id>`
- `-d`
  - Detached mode
  - Runs the container in the background
- `--mount type=bind, source=<abs_host_path>, target=<abs_container_path>`
  - Mounts a volume from the host machine to the container
  - `type`
    - Is always `bind` for this command (can be omitted)
  - `source`
    - Path to the file or directory on the Docker daemon host
    - May be specified as `source` or `src`.
    - $(pwd) can be used to get the current directory
  - `target`
    - Path to the file or directory in the container
    - May be specified as `target` or `dst`.
- `-v <abs_host_path>:<abs_container_path>`
  - Mounts a volume from the host machine to the container
  - These paths need to be absolute paths
  - `$PWD` can be used to get the current directory
    - "Print Working Directory"
- `--network=<network_name>`
  - Connects the container to a network
  - Can be set as `host` to connect to the host network
- `--ipc=<ipc_mode>`
  - "Inter-Process Communication"
  - Sets the IPC mode for the container
  - Can be set as `host` to use the host's IPC namespace

### `ps`

- `container ls`

Prints all of the running containers.

#### `ps` Optional parameters

- `-a`
  - Shows running and stopped containers.

### `start <container_name>`

- `container start <container_name>`

Starts the specified container.\
Has the same optional parameters as the [run](#run-optional-parameters) command.

### `stop <container_name>`

- `container start <container_name>`

Stops the specified container from running.

### `exec <container_name> <excutable_command>`

- `container exec <container_name> <excutable_command>`

Has the same optional parameters as the [run](#run-optional-parameters) command.

The executable command is the command that you want to run in the container. This could be as simple as `ls` or `pwd` or something more complex.

### `rm <container_name>`

- `container rm <container_name>`

Removes the specified container.

### `container prune`

Removes all except the running containers
