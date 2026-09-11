#!/usr/bin/env bash
# =============================================================================
# install_datasets.sh - extract already-downloaded dataset archives into ~/ORB_SLAM3/datasets/
# =============================================================================
# Detected layout: fr1_360.gz / fr1_floor.gz / fr1_room.gz (the three missing TUM sequences) + V1_02_medium.zip (EuRoC)
# Usage (WSL):
#   cd ~/ORB_SLAM3 && bash "$W/install_datasets.sh"
# Afterwards:
#   TUM remaining runs:  bash "$W/run_v2_tum.sh" sel fr1_room fr1_360 fr1_floor
#   EuRoC:               after downloading the remaining sequences, bash "$W/run_v2_euroc.sh"
# =============================================================================
set -uo pipefail
ORB="${ORB_SLAM3_ROOT:-$HOME/ORB_SLAM3}"
cd "$ORB" || { echo "[ERROR] $ORB not found"; exit 1; }
SRC="${DATASET_SRC:-$HOME/datasets_download}"
[ -d "$SRC" ] || { echo "[ERROR] $SRC not found (set DATASET_SRC to your download folder)"; exit 1; }

mkdir -p datasets

echo "==== 1) TUM RGB-D (fr1_room / fr1_360 / fr1_floor) ===="
for f in rgbd_dataset_freiburg1_room.gz rgbd_dataset_freiburg1_360.gz rgbd_dataset_freiburg1_floor.gz; do
  name="${f%.gz}"
  if [ -d "datasets/$name" ]; then
    echo "  [skip] $name already exists"
  elif [ -f "$SRC/$f" ]; then
    echo "  [extract] $f ..."
    tar -xzf "$SRC/$f" -C datasets/
    [ -d "datasets/$name" ] && echo "  [ok] $name" || echo "  [warn] datasets/$name not found after extraction"
  else
    echo "  [warn] $SRC/$f does not exist"
  fi
done

echo ""
echo "==== 2) EuRoC MAV (downloaded zips) ===="
mkdir -p datasets/euroc
# 2a) Standard EuRoC zips downloaded directly (V1_02_medium.zip / MH_01_easy.zip ...)
#     The official archive has mav0 at the top level, so extract into a directory of the same name: datasets/euroc/V1_02_medium/mav0
for f in "$SRC"/*.zip; do
  [ -f "$f" ] || continue
  b=$(basename "$f")
  case "$b" in
    V*_*.zip|MH*_*.zip)
      seq="${b%.zip}"
      if [ -d "datasets/euroc/$seq/mav0" ]; then
        echo "  [skip] $seq already present"
      else
        mkdir -p "datasets/euroc/$seq"
        echo "  [unzip] $b -> datasets/euroc/$seq/"
        unzip -q -o "$f" -d "datasets/euroc/$seq"
        [ -d "datasets/euroc/$seq/mav0" ] && echo "  [ok] $seq ready" || echo "  [warn] mav0 not found after extracting $seq"
      fi
      ;;
  esac
done
# 2b) Bundle archives (e.g. vicon_room1.zip contains per-sequence .zip files plus .bag files)
#     Unpack first, then apply the 2a handling to every standard EuRoC zip inside
for f in "$SRC"/*.zip; do
  [ -f "$f" ] || continue
  b=$(basename "$f")
  case "$b" in
    V*_*.zip|MH*_*.zip) continue ;;
  esac
  echo "  [bundle] $b -> datasets/euroc/ (looking for nested EuRoC zips)"
  unzip -q -o "$f" -d datasets/euroc/
done
# 2c) Extract each nested standard EuRoC zip into a directory of the same name (V1_02_medium.zip -> datasets/euroc/V1_02_medium/mav0)
for nz in datasets/euroc/*/*.zip; do
  [ -f "$nz" ] || continue
  nb=$(basename "$nz")
  case "$nb" in
    V*_*.zip|MH*_*.zip)
      seq="${nb%.zip}"
      if [ -d "datasets/euroc/$seq/mav0" ]; then
        echo "  [skip] $seq already present"
      else
        mkdir -p "datasets/euroc/$seq"
        echo "  [unzip] $nb -> datasets/euroc/$seq/"
        unzip -q -o "$nz" -d "datasets/euroc/$seq"
        [ -d "datasets/euroc/$seq/mav0" ] && echo "  [ok] $seq ready" || echo "  [warn] mav0 not found after extracting $seq"
      fi
      ;;
  esac
done
echo ""
echo "==== 3) Summary ===="
echo "-- TUM datasets --"
ls -d datasets/rgbd_dataset_freiburg* 2>/dev/null || echo "  (none)"
echo "-- EuRoC datasets (directories containing mav0) --"
for d in datasets/euroc/*/; do
  [ -d "$d/mav0" ] && echo "  ${d%/}"
done
echo ""
echo "-- EuRoC sequences still to download --"
for s in MH01 MH02 MH03 MH04 MH05 V101 V102 V103 V201 V202 V203; do
  found=""
  for d in datasets/euroc/*/; do
    b=$(basename "$d"); norm=$(printf '%s' "$b" | tr -cd '[:alnum:]')
    case "$norm" in "$s"*) found=1;; esac
  done
  [ -z "$found" ] && echo "  $s"
done
echo ""
echo "Download: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets"
echo "TUM remaining runs: bash \"\$W/run_v2_tum.sh\" sel fr1_room fr1_360 fr1_floor"