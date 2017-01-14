    sudo apt-get install libeigen3-dev
    cd build
    cmake ../
    make


use gnuplot and its C++ interface
    sudo apt-get install gnuplot gnuplot-x11

this one depends on boost, to get boost version 
    dpkg -s libboost-dev | grep 'Version'

undefined reference to boost::iostreams::file_descriptor::seek

because we need to add boost component iostream

killall gnuplot
