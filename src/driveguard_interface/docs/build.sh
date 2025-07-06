#!/bin/bash
echo "⏩ 构建文档......"

# 清理构建目录
rm -rf build

# 确保源目录结构
mkdir -p source/_static

# 生成 API 文档
sphinx-apidoc -o source ../driveguard_interface

# 构建 HTML 文档
make html
echo "✅ 文档构建成功!"

echo "⏩ 启动本地服务器查看文档......"
python3 -m http.server 8000 --directory build/html
