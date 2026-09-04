#!/usr/bin/env bash
# =============================================================================
# install_datasets.sh - 把 D:\dateset 已下载的数据解压到 WSL ~/ORB_SLAM3/datasets/
# =============================================================================
# 已检测到: fr1_360.gz / fr1_floor.gz / fr1_room.gz (TUM 缺的三个) + V1_02_medium.zip (EuRoC)
# 用法 (WSL):
#   cd ~/ORB_SLAM3 && bash "$W/install_datasets.sh"
# 之后:
#   TUM 补跑:  bash "$W/run_v2_tum.sh" sel fr1_room fr1_360 fr1_floor
#   EuRoC:     下载其余序列后 bash "$W/run_v2_euroc.sh"
# =============================================================================
set -uo pipefail
ORB="${ORB_SLAM3_ROOT:-$HOME/ORB_SLAM3}"
cd "$ORB" || { echo "[ERROR] $ORB 不存在"; exit 1; }
SRC="${DATASET_SRC:-/mnt/d/0 科研学习/SLAM/dateset}"
[ -d "$SRC" ] || { echo "[ERROR] 找不到 $SRC (D 盘未挂载?)"; exit 1; }

mkdir -p datasets

echo "==== 1) TUM RGB-D (fr1_room / fr1_360 / fr1_floor) ===="
for f in rgbd_dataset_freiburg1_room.gz rgbd_dataset_freiburg1_360.gz rgbd_dataset_freiburg1_floor.gz; do
  name="${f%.gz}"
  if [ -d "datasets/$name" ]; then
    echo "  [skip] $name 已存在"
  elif [ -f "$SRC/$f" ]; then
    echo "  [extract] $f ..."
    tar -xzf "$SRC/$f" -C datasets/
    [ -d "datasets/$name" ] && echo "  [ok] $name" || echo "  [warn] 解压后未找到 datasets/$name"
  else
    echo "  [warn] $SRC/$f 不存在"
  fi
done

echo ""
echo "==== 2) EuRoC MAV (已下载的 zip) ===="
mkdir -p datasets/euroc
# 2a) 直接下载的标准 EuRoC zip (V1_02_medium.zip / MH_01_easy.zip ...)
#     官方包顶层就是 mav0, 需解压到同名目录: datasets/euroc/V1_02_medium/mav0
for f in "$SRC"/*.zip; do
  [ -f "$f" ] || continue
  b=$(basename "$f")
  case "$b" in
    V*_*.zip|MH*_*.zip)
      seq="${b%.zip}"
      if [ -d "datasets/euroc/$seq/mav0" ]; then
        echo "  [skip] $seq 已就绪"
      else
        mkdir -p "datasets/euroc/$seq"
        echo "  [unzip] $b -> datasets/euroc/$seq/"
        unzip -q -o "$f" -d "datasets/euroc/$seq"
        [ -d "datasets/euroc/$seq/mav0" ] && echo "  [ok] $seq 就绪" || echo "  [warn] $seq 解压后未找到 mav0"
      fi
      ;;
  esac
done
# 2b) bundle 型打包包 (如 vicon_room1.zip: 内含各序列的 .zip + .bag)
#     先解包, 再对里面每个标准 EuRoC zip 做 2a 的处理
for f in "$SRC"/*.zip; do
  [ -f "$f" ] || continue
  b=$(basename "$f")
  case "$b" in
    V*_*.zip|MH*_*.zip) continue ;;
  esac
  echo "  [bundle] $b -> datasets/euroc/ (查找嵌套 EuRoC zip)"
  unzip -q -o "$f" -d datasets/euroc/
done
# 2c) 把解出的嵌套标准 EuRoC zip 各解压到同名目录 (V1_02_medium.zip -> datasets/euroc/V1_02_medium/mav0)
for nz in datasets/euroc/*/*.zip; do
  [ -f "$nz" ] || continue
  nb=$(basename "$nz")
  case "$nb" in
    V*_*.zip|MH*_*.zip)
      seq="${nb%.zip}"
      if [ -d "datasets/euroc/$seq/mav0" ]; then
        echo "  [skip] $seq 已就绪"
      else
        mkdir -p "datasets/euroc/$seq"
        echo "  [unzip] $nb -> datasets/euroc/$seq/"
        unzip -q -o "$nz" -d "datasets/euroc/$seq"
        [ -d "datasets/euroc/$seq/mav0" ] && echo "  [ok] $seq 就绪" || echo "  [warn] $seq 解压后未找到 mav0"
      fi
      ;;
  esac
done
echo ""
echo "==== 3) 汇总 ===="
echo "-- TUM 数据集 --"
ls -d datasets/rgbd_dataset_freiburg* 2>/dev/null || echo "  (无)"
echo "-- EuRoC 数据集 (含 mav0 的目录) --"
for d in datasets/euroc/*/; do
  [ -d "$d/mav0" ] && echo "  ${d%/}"
done
echo ""
echo "-- EuRoC 仍需下载 --"
for s in MH01 MH02 MH03 MH04 MH05 V101 V102 V103 V201 V202 V203; do
  found=""
  for d in datasets/euroc/*/; do
    b=$(basename "$d"); norm=$(printf '%s' "$b" | tr -cd '[:alnum:]')
    case "$norm" in "$s"*) found=1;; esac
  done
  [ -z "$found" ] && echo "  $s"
done
echo ""
echo "下载地址: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets"
echo "TUM 补跑: bash \"\$W/run_v2_tum.sh\" sel fr1_room fr1_360 fr1_floor"