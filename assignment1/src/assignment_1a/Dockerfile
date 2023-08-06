# Build
FROM ubuntu:22.04 as builder
RUN apt-get update && \
      apt-get install -y \
      ca-certificates \
      cmake \
      g++ \
      make \
      software-properties-common
RUN add-apt-repository -y ppa:chrberger/libcluon && \
  apt-get update && apt-get install -y libcluon
  #https://chrberger.github.io/libcluon/ubuntu/v20.04 --allow-untrusted
ADD . /opt/sources
WORKDIR /opt/sources
RUN mkdir /tmp/build && cd /tmp/build && \
  cmake /opt/sources && \
  make && make test && cp helloworld /tmp

# Deploy
FROM ubuntu:22.04
RUN apt-get update && \
  apt-get install -y \
    libstdc++6
COPY --from=builder /tmp/helloworld /usr/bin
CMD ["/usr/bin/helloworld"]
