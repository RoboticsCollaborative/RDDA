# RDDA

## Dependency

### Install SOEM
`git clone https://github.com/OpenEtherCATsociety/SOEM`

`mkdir build`

`cd build`

`cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local/`

`make`

`sudo make install`

## Install RDDA
`mkdir build`

`cd build`

`cmake ..`

`make`


### Grant socket permission to executable
` sudo setcap cap_net_admin,cap_net_raw=eip ./rdda_slave`  (or rdda_master)