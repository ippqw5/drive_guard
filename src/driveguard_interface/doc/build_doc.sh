#!/bin/bash

rm -rf build

# 删除source下除了index.rst和conf.py之外的所有文件
mv source/index.rst ./
mv source/conf.py ./
rm -rf source/*
mv conf.py source/
mv index.rst source/

# 生成API文档
sphinx-apidoc -o source ../driveguard_interface/

make html