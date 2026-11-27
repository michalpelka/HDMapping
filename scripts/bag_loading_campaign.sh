#!/usr/bin/env bash
# Loading test campaign for ROS1 bags and MCAP files in lidar_odometry_step_1: downloads public recordings, writes them
# in every container and compression, and times step 1's loading of each. See bag_loading_campaign.py --help.
#
# Runs in bash on Linux, macOS and MSYS2 on Windows. Needs Python 3.10 or newer; on Windows the python.org Python
# (the "py" launcher) is used, because native executables need Windows paths and wheels exist for its packages.
#
#   scripts/bag_loading_campaign.sh                       # download, prepare and run, data in ~/hdmapping_bag_campaign
#   scripts/bag_loading_campaign.sh --data /d/campaign run --variants ros1_bz2 mcap_zstd
#   scripts/bag_loading_campaign.sh --list
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# the Python environment lives in the data folder, next to the downloads
data="$HOME/hdmapping_bag_campaign"
args=("$@")
for ((i = 0; i < ${#args[@]}; i++)); do
    if [[ "${args[$i]}" == "--data" && $((i + 1)) -lt ${#args[@]} ]]; then
        data="${args[$((i + 1))]}"
    fi
done
mkdir -p "$data"

case "$(uname -s)" in
    MINGW* | MSYS* | CYGWIN*) candidates=("py -3" python python3) ;;
    *) candidates=(python3 python) ;;
esac
python=""
for candidate in "${candidates[@]}"; do
    if $candidate -c 'import sys; sys.exit(sys.version_info < (3, 10))' 2>/dev/null; then
        python="$candidate"
        break
    fi
done
if [[ -z "$python" ]]; then
    echo "Python 3.10 or newer not found (on Windows install it from python.org)" >&2
    exit 1
fi

venv="$data/.venv"
if [[ ! -d "$venv" ]]; then
    echo "Creating a Python environment in $venv"
    $python -m venv "$venv"
fi
if [[ -x "$venv/Scripts/python.exe" ]]; then
    venv_python="$venv/Scripts/python.exe"
else
    venv_python="$venv/bin/python"
fi
if ! "$venv_python" -c 'import rosbags, mcap, lz4, zstandard' 2>/dev/null; then
    "$venv_python" -m pip install --quiet --upgrade pip
    "$venv_python" -m pip install --quiet rosbags mcap lz4 zstandard
fi

exec "$venv_python" "$here/bag_loading_campaign.py" "$@"
