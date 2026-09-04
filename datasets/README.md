# Datasets

The benchmark datasets used in the paper are **not** included in this
repository (size + license terms). Download them from the official sources:

| Benchmark | Sequences | Source |
|---|---|---|
| TUM RGB-D | freiburg1_desk, freiburg1_desk2, freiburg1_room, freiburg1_360, freiburg1_floor, freiburg2_xyz, freiburg2_desk, freiburg3_long_office_household, freiburg3_sitting_xyz | https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download |
| EuRoC MAV (ASL format) | V1_01_easy, V1_02_medium, V1_03_difficult | https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets |
| KITTI odometry | 00, 02, 05, 07, 09, 10 | https://www.cvlibs.net/datasets/kitti/eval_odometry.php |

## Expected layout

Place the extracted data inside the ORB-SLAM3 tree:

```
$ORB_SLAM3_ROOT/datasets/
  rgbd_dataset_freiburg1_desk/
  rgbd_dataset_freiburg1_desk2/
  ...
  euroc/V1_01_easy/mav0/
  euroc/V1_02_medium/mav0/
  euroc/V1_03_difficult/mav0/
  kitti/dataset/sequences/{00,02,05,07,09,10}/
  kitti/dataset/poses/
```

## install_datasets.sh

`install_datasets.sh` is a convenience script written for the author's WSL
setup (it extracts archives already downloaded to a local folder, configured
via the `DATASET_SRC` environment variable, default
`/mnt/d/0 科研学习/SLAM/dateset`). Reviewers can ignore it and extract the
official archives directly into the layout above.

Respect the original licenses of the datasets when downloading and using them.
