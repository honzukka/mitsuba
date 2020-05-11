FROM ubuntu:14.04

LABEL author="Jan Horesovsky <jan.horesovsky@aalto.fi>"
LABEL based_on="https://github.com/benjamin-heasly/mitsuba-docker"

# INSTALL MITSUBA DEPENDENCIES
# ----------------------------------------------------
RUN apt-get update \
    && apt-get install -y \
    build-essential \
    scons \
    git \
    libpng12-dev \
    libjpeg-dev \
    libilmbase-dev \
    libxerces-c-dev \
    libboost-all-dev \
    libopenexr-dev \
    libglewmx-dev \
    libxxf86vm-dev \
    libpcrecpp0 \
    libeigen3-dev \
    libfftw3-dev \
    wget \
    && apt-get clean \
    && apt-get autoclean \
    && apt-get autoremove

WORKDIR /mitsuba

# BUILD A CUSTOM VERSION OF MITSUBA
# ----------------------------------------------------
RUN git clone https://github.com/honzukka/mitsuba.git
WORKDIR /mitsuba/mitsuba

# disable GUI build
RUN sed -i '54s/\(.*\)/#\1/' SConstruct
RUN sed -i '54s/\(.*\)/#\1/;55s/\(.*\)/#\1/' build/SConscript.install

RUN cp build/config-linux-gcc.py config.py \
    && scons

# SET UP ENV VARS (like in Mitsuba's own setpath.sh)
# ----------------------------------------------------
ENV MITSUBA_DIR /mitsuba/mitsuba
ENV PYTHONPATH /mitsuba/mitsuba/dist/python:/mitsuba/mitsuba/dist/python/2.7:
ENV PATH /mitsuba/mitsuba/wrapper:/mitsuba/mitsuba/dist:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
ENV LD_LIBRARY_PATH /mitsuba/mitsuba/dist:

# fix for locale issues in some clusters (from: https://github.com/benjamin-heasly/mitsuba-docker/pull/2)
RUN locale-gen en_US.UTF-8  
ENV LANG en_US.UTF-8  
ENV LANGUAGE en_US:en  
ENV LC_ALL en_US.UTF-8