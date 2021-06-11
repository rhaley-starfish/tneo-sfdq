Setting up in CLion

This will explain how to setup with openocd, cmake, ninja, and clion.

###Install CLion 

I just grab it from jetbrains.com

###Install CMake
   Please use at least 3.19. The general instruction is to grab it from https://cmake.org/download/.
####Windows 
To set up on windows, download cmake from cmake.com or use choco. 
####Linux 
You can grab a shell installer from cmake.org as noted above. 
Source build:
```bash
sudo apt install build-essential libssl-dev
wget https://github.com/Kitware/CMake/releases/download/v3.20.2/cmake-3.20.2.tar.gz
tar -zxvf cmake-3.20.2.tar.gz
cd cmake-3.20.2
./bootstrap
make 
sudo make install 
```
Other options here: https://graspingtech.com/upgrade-cmake/

Using apt is a little complicated because you have to add the kitware repository. However that means you also get the latest version when you update using apt.
https://apt.kitware.com/


### Install Ninja
Generally ninja is up to date on most platforms using a package manager. You can manually install it here: https://github.com/ninja-build/ninja/releases

### OpenOCD
OpenOCD installed and worked correctly with apt and homebrew but did not work on windows with choco. Binaries can be downloaded for Windows at https://gnutoolchains.com/arm-eabi/openocd/. Add the bin folder to your path.

Windows

*Ensure you have the correct drivers. If using an ST-Link debugger, download the Windows drivers: https://www.st.com/en/development-tools/stsw-link009.html
I can't remember if I had to do something specific in Linux or on Mac.

