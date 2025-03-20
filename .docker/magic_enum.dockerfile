RUN mkdir -p /tmp/vendor && cd /tmp/vendor && wget -c https://github.com/Neargye/magic_enum/archive/refs/tags/v0.8.0.tar.gz -O  magic_enum-0.8.0.tar.gz &&\
    tar -xzvf magic_enum-0.8.0.tar.gz &&\
    cd magic_enum-0.8.0 &&\
    mkdir build && cd build &&\
    cmake .. && make -j4 &&\
    sudo make install &&\
    sudo ldconfig &&\
    cd ../.. && rm -r magic_enum*   