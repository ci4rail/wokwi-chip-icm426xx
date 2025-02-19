FROM wokwi/builder-clang-wasm

USER root
RUN apk add bash gcompat libc6-compat curl

# Install arduino-cli:
RUN  wget -O /tmp/cli.tar.gz \
        https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Linux_64bit.tar.gz && \
    tar -C /usr/local/bin -zxvf /tmp/cli.tar.gz && \
    rm /tmp/cli.tar.gz
RUN arduino-cli core update-index && arduino-cli core install arduino:avr

RUN curl -L https://wokwi.com/ci/install.sh | sh