#!/bin/bash

ENV=wcb-test.bin.env
make clean

rm -f $ENV

pwd 2>&1 | tee -a $ENV
msp430-gcc --version 2>&1 | tee -a $ENV
export 2>&1 | tee -a $ENV

echo "-- git status -------" >> $ENV
git log -1 --format="%H" >> $ENV
git --no-pager status --porcelain 2>&1 | tee -a $ENV
git --no-pager diff >> $ENV

echo "-- build log --------" >> $ENV
make 2>&1 | tee -a $ENV
