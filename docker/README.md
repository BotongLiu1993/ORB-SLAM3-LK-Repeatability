# Docker

The original ORB-SLAM3 repository and this study's code snapshots do not
contain a Dockerfile. `Dockerfile.example` is a minimal, UNVALIDATED example
for reviewers who want a containerized build environment. It was not used to
produce any number in the paper; all experiments ran natively in WSL
(Ubuntu 20.04, 12 cores, viewer disabled).

To use it: place the ORB-SLAM3 snapshot (with `S5_lk_v2.patch` applied) in
this directory and run `docker build -t orbslam3-lkv2 .`.
