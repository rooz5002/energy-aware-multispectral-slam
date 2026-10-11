#!/bin/bash
# Fetch ORB-SLAM3 at the pinned commit, apply the multispectral patch, build it and the Python binding.
#   orbslam3_patch/build.sh
set -e
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMMIT=4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4   # UZ-SLAMLab/ORB_SLAM3 master, v1.0 (2022-02-10)
SRC="$REPO/third_party/ORB_SLAM3"
FLAGS="-DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5"
JOBS=${JOBS:-8}

if [ ! -d "$SRC/.git" ]; then
    mkdir -p "$SRC" && cd "$SRC" && git init -q && git remote add origin https://github.com/UZ-SLAMLab/ORB_SLAM3.git
    git fetch -q --depth 1 origin "$COMMIT" && git checkout -q FETCH_HEAD
    git apply "$REPO/orbslam3_patch/orbslam3_ms.patch"
fi
cd "$SRC"
[ "$(git rev-parse HEAD)" = "$COMMIT" ] || { echo "ORB-SLAM3 is not at the pinned commit"; exit 1; }
for d in DBoW2 g2o; do
    cmake -S Thirdparty/$d -B Thirdparty/$d/build $FLAGS > /dev/null && cmake --build Thirdparty/$d/build -j"$JOBS"
done
[ -f Vocabulary/ORBvoc.txt ] || tar -xf Vocabulary/ORBvoc.txt.tar.gz -C Vocabulary
cmake -S . -B build $FLAGS > /dev/null && cmake --build build -j"$JOBS"
# binding for the project's virtual environment (ROS leaks PYTHONPATH, hence env -u)
env -u PYTHONPATH cmake -S "$REPO/orbslam3_patch/python" -B "$REPO/orbslam3_patch/python/build" \
    -DPython_EXECUTABLE="$REPO/.venv/bin/python" -DPYBIND11_FINDPYTHON=ON > /dev/null
cmake --build "$REPO/orbslam3_patch/python/build" -j"$JOBS"
echo "built: $(ls "$REPO"/harness/orbslam3_ms*.so)"
