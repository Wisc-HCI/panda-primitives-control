
FROM osrf/ros:noetic-desktop-full

# Set noninteractive to avoid prompts during the build
ARG DEBIAN_FRONTEND=noninteractive

# Update apt package list and install general packages
RUN apt-get update && \
    apt-get install -y --fix-missing \
    curl wget \
    nano \
    build-essential \ 
    cmake \
    libeigen3-dev\
    python3-catkin-tools \
    ros-noetic-libfranka ros-noetic-franka-ros \
    ros-noetic-urdfdom-py \
    ros-noetic-kdl-parser-py \
    ros-noetic-kdl-conversions\
    ros-noetic-spacenav-node\
    usbutils\
    python3-pip

# Add python alias to python3
RUN ln -s /usr/bin/python3 /usr/bin/python
RUN pip3 install scipy

# Setup spacenavd (can't use apt-get bc that version is too old)
WORKDIR /workspace/
COPY . .
WORKDIR /workspace/spacenavd
RUN ./configure
RUN make install
WORKDIR /workspace/



# # Install Julia for RelaxedIK
# WORKDIR /
# RUN wget https://julialang-s3.julialang.org/bin/linux/x64/1.8/julia-1.8.1-linux-x86_64.tar.gz \
#     && tar zxvf julia-1.8.1-linux-x86_64.tar.gz
# ENV PATH="/julia-1.8.1/bin:${PATH}"
# # TODO: Perhaps move julia into another location besides root

# # Install Julia dependencies
# RUN julia -e 'import Pkg; Pkg.add(Pkg.PackageSpec(name="Rotations", version="0.13.0"))'
# RUN julia -e 'import Pkg; Pkg.add(["YAML", "BenchmarkTools", "ForwardDiff", "Calculus", "ReverseDiff", "StaticArrays", "Flux", "BSON", "NLopt", "Knet", "Random", "RobotOS", "Distributions", "PyCall", "Dates", "LinearAlgebra", "Zygote", "Distances"]) '
# ENV PYTHON=/usr/bin/python2.7
# RUN julia -e 'import Pkg; Pkg.add("PyCall"); Pkg.build("PyCall")'

# # Install Python dependencies
# RUN wget https://bootstrap.pypa.io/pip/2.7/get-pip.py
# RUN python2.7 get-pip.py
# RUN python2.7 -m pip install --upgrade pip
# RUN python2.7 -m pip install \
#     readchar \
#     python-fcl \
#     scikit-learn \
#     scipy \
#     PyYaml \
#     rospkg
    
# RUN python2.7 -m pip install --upgrade numpy




# Set the default command to execute
CMD ["bash"]



