# Setup — Ubuntu 24.04 under WSL2

From a fresh WSL2 Ubuntu 24.04 to a recon log in hand. Everything below runs
inside the WSL terminal, not PowerShell.

## Step 0 — confirm you're on Noble

```bash
lsb_release -a
```

Must say `24.04` / `noble`. If it says 22.04 you have the wrong WSL distro
installed; see "Wrong Ubuntu version" at the bottom.

## Step 1 — create the project folder in the Linux filesystem

```bash
mkdir -p ~/ur5e_pickplace
cd ~/ur5e_pickplace
```

**Do not put the project under `/mnt/c/`.** This matters more than it sounds:

- WSL2 reaches Windows drives over a 9p network filesystem. `colcon build` does
  tens of thousands of small file operations and will be roughly an order of
  magnitude slower there.
- The Windows mount doesn't carry Unix permissions by default, so `chmod +x`
  silently does nothing and your scripts won't run.
- Case-insensitivity on NTFS breaks some ROS packages that ship files differing
  only by case.

`~` is the real ext4 filesystem inside the WSL VM. Use it.

## Step 2 — get the project files in

If the files are on the Windows side, WSL sees `C:\` as `/mnt/c`:

```bash
cp -r /mnt/c/Users/<you>/path/to/project/. ~/ur5e_pickplace/
cd ~/ur5e_pickplace
ls
```

You should see `README.md`, `config/`, `docs/`, `scripts/`.

Cloning from a remote instead is fine, and avoids the line-ending problem in
step 3 entirely:

```bash
git clone <remote> ~/ur5e_pickplace
```

## Step 3 — make the scripts executable and check line endings

```bash
cd ~/ur5e_pickplace
chmod +x scripts/*.sh

# Everything here uses Unix line endings, but a round trip through a Windows
# editor can add CRs. A stray \r makes bash fail with the unhelpful
# "bad interpreter: /usr/bin/env bash^M".
file scripts/*.sh          # want "ASCII text", NOT "with CRLF line terminators"
sed -i 's/\r$//' scripts/*.sh config/*.yaml     # harmless if already clean
```

## Step 4 — git init

```bash
cd ~/ur5e_pickplace
git init
git add -A
git commit -m "M-1 kit: reference report, scene config, recon and M0 verification"
```

## Step 5 — install the stack

```bash
bash scripts/02_bootstrap_noble.sh 2>&1 | tee bootstrap_$(date +%Y%m%d_%H%M%S).log
```

Expect 20–40 minutes and several sudo prompts. Exit code 0 means verified; any
`[FAIL]` line means stop and read it.

WSL2 defaults to a memory cap that a parallel MoveIt build can exceed, so
colcon can get OOM-killed. Either limit parallelism:

```bash
colcon build --symlink-install --parallel-workers 2
```

or raise the cap by creating `C:\Users\<you>\.wslconfig` on the Windows side:

```ini
[wsl2]
memory=12GB
processors=8
```

then `wsl --shutdown` from PowerShell and reopen the terminal.

## Step 6 — Gazebo rendering smoke test (WSL-specific)

Before touching the robot, confirm Gazebo can render at all:

```bash
source /opt/ros/jazzy/setup.bash
gz sim shapes.sdf
```

A window with shapes should appear via WSLg.

**If it fails or is unusably slow**, Gazebo Harmonic's default OGRE2 renderer
isn't getting hardware acceleration. In order of preference:

```bash
# 1. Check what GL driver you actually got
sudo apt install -y mesa-utils
glxinfo -B | grep -E 'OpenGL renderer|OpenGL version'
#    "D3D12 (...)" = GPU passthrough working
#    "llvmpipe"    = pure software, will be very slow

# 2. Fall back to the older renderer, far more tolerant under WSL
gz sim --render-engine ogre shapes.sdf

# 3. Last resort — force software rendering (correct, but slow)
export LIBGL_ALWAYS_SOFTWARE=1
```

Make sure your **Windows** GPU driver is current. WSL gets GPU access through
the Windows driver, so updating inside Ubuntu does nothing.

### Run milestone data collection headless

Every milestone's evidence — the 20/20 planning log, TCP pose against
commanded, the M3 slip CSV, the M5 repeatability run — comes from state topics,
not from pixels. So run the sim headless for data and open the GUI only for
spot checks:

```bash
gz sim -s <world>          # server only, no GUI
```

Headless sidesteps WSL rendering entirely and runs considerably faster, which
matters when M3 and M5 each want 20 consecutive cycles. Use the GUI when you
need to *see* something, and take screenshots from a GUI run. Note in the log
which run a screenshot came from, since the spec requires a screenshot and its
claims to be reconcilable against the same run.

## Step 7 — arm-only sanity check

Before adding a gripper, confirm UR's own supported path works:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur5e_ws/install/setup.bash          # only if bootstrap built from source
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur5e
```

You want RViz with a UR5e and a working MoveIt planning session. **If this
doesn't come up, stop here.** The problem is upstream of anything this project
adds, and every later milestone would inherit it.

## Step 8 — recon

```bash
cd ~/ur5e_pickplace
source /opt/ros/jazzy/setup.bash
mkdir -p docs
bash scripts/00_recon.sh 2>&1 | tee docs/recon_$(date +%Y%m%d_%H%M%S).log
```

Read-only: installs nothing, launches no simulation.

It answers the five open questions in `docs/M-1_reference_report.md` §6: the UR
macro signature, the robotiq package source, the flange frame and mount
transform, the actuated gripper joint's real limits, and the five mimic
multipliers. The URDF merge is written from those observed values rather than
from the donor repo's README.

## Troubleshooting

**Wrong Ubuntu version.** WSL's default `Ubuntu` distro may be 22.04. Install
Noble explicitly from PowerShell:

```powershell
wsl --install -d Ubuntu-24.04
wsl -l -v                  # confirm VERSION 2, not 1
```

WSL **1** will not work: it has no real kernel, and Gazebo's physics and
networking assumptions break. Use `wsl --set-version Ubuntu-24.04 2` if needed.

**`ros2` not found in a new terminal.** The bootstrap appends the setup line to
`~/.bashrc`, which only applies to terminals opened afterwards. Open a new one
or `source ~/.bashrc`.

**Clock skew after Windows sleep.** WSL's clock drifts when the host suspends,
which makes apt reject repository signatures and can upset ROS timestamps:

```bash
sudo hwclock -s
```
