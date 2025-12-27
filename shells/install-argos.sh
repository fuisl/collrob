sudo apt-get update

# install dependencies
sudo apt-get install -y \
  git build-essential cmake \
  libfreeimage-dev libfreeimageplus-dev \
  qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools \
  freeglut3-dev libxi-dev libxmu-dev \
  liblua5.3-dev lua5.3 \
  doxygen graphviz libgraphviz-dev asciidoc

# compile argos3 from source
cd ~
git clone https://github.com/ilpincy/argos3.git
cd argos3
mkdir -p build_simulator
cd build_simulator
cmake ../src
make -j"$(nproc)"

# install system-wide
make doc
sudo make install
sudo ldconfig