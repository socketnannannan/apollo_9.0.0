ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG GEOLOC
ARG CLEAN_DEPS
ARG INSTALL_MODE

WORKDIR /apollo

COPY archive /tmp/archive

COPY installers /opt/apollo/installers

RUN bash /opt/apollo/installers/install_geo_adjustment.sh ${GEOLOC}
RUN bash /opt/apollo/installers/install_modules_base.sh
RUN bash /opt/apollo/installers/install_gpu_support.sh
RUN bash /opt/apollo/installers/install_ordinary_modules.sh
RUN bash /opt/apollo/installers/install_drivers_deps.sh ${INSTALL_MODE}
RUN bash /opt/apollo/installers/install_dreamview_deps.sh ${GEOLOC}
RUN bash /opt/apollo/installers/install_contrib_deps.sh

RUN bash /opt/apollo/installers/install_release_deps.sh
RUN bash /opt/apollo/installers/post_install.sh ${BUILD_STAGE}
RUN bash /opt/apollo/installers/install_rsdriver.sh
