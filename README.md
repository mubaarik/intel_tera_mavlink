# intel_tera_mavlink

## Installation
#### Installing Serial Lib
`git clone https://github.com/wjwwood/serial.git`
`cd serial`

#### Installing the driver

`git clone https://github.com/mubaarik/intel_tera_mavlink.git`
`cd intel_tera_mavlink`
`git submodule update --init --recursive`
`mkdir build`
`cd build`
`cmake ..`
`make`
`sudo make install`
`sudo cp aero-teraranger.service /lib/systemmd/system/`
`sudo cp intel_tera_mavlink /usr/bin`
`sudo system`

