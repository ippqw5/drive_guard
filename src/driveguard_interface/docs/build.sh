#!/bin/bash

# 清理构建目录
rm -rf build

# 确保源目录结构
mkdir -p source/_static

# 生成 API 文档
sphinx-apidoc -o source ../src/driveguard

# 构建 HTML 文档
make html

echo "Documentation built successfully!"
echo "Open build/html/index.html to view the documentation."