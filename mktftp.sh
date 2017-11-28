#!/bin/bash

#export PATH=~/work/gnutools/arm-linaro4-x86/bin:${PATH}

make UIMAGE_COMPRESSION=lzma uImage $1
cp arch/arm/boot/uImage /tftpboot/.

