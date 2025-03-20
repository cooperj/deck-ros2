RUN mkdir -p /tmp/vendor && cd /tmp/vendor && wget -c https://github.com/google/glog/archive/refs/tags/v0.6.0.tar.gz  -O glog-0.6.0.tar.gz &&\
    tar -xzvf glog-0.6.0.tar.gz &&\
    cd glog-0.6.0 &&\
    mkdir build && cd build &&\
    cmake .. && make -j4 &&\
    sudo make install &&\
    sudo ldconfig &&\
    cd ../.. && rm -r glog-*