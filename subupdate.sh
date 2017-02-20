#!/bin/sh
echo "--------------- start to update submodules: ----------------"
git submodule update --init --recursive
echo "--------------- submodules update succeed! -----------------"
