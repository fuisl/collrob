argos3 --version

cd ~
git clone https://github.com/ilpincy/argos3-examples.git
cd argos3-examples
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j"$(nproc)"

cd ~/argos3-examples
argos3 -c experiments/diffusion_1.argos
