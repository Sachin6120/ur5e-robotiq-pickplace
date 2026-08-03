# Setup — Ubuntu 24.04 under WSL2

From a fresh WSL2 Ubuntu 24.04 to "recon log in hand". Run everything below
inside the WSL terminal, not PowerShell.

---

## Step 0 — confirm you're actually on Noble

```bash
lsb_release -a
```

Must say `24.04` / `noble`. If it says 22.04, you have the wrong WSL distro
installed — see "Wrong Ubuntu version" at the bottom.

---

## Step 1 — create the project folder in the LINUX filesystem

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

---

## Step 2 — copy the kit across from the Cowork folder

The files live on the Windows side. WSL sees `C:\` as `/mnt/c`.

```bash
# Find it (the session ID changes between Cowork sessions, so search rather
# than trusting a hardcoded path):
KIT=$(find /mnt/c/Users/sachi/AppData/Roaming/Claude/local-agent-mode-sessions \
        -type d -name outputs 2>/dev/null \
      | xargs -I{} sh -c 'test -f "{}/scripts/00_recon.sh" && echo {}' \
      | head -1)

echo "found kit at: $KIT"
```

If that prints a path, copy it in:

```bash
cp -r "$KIT"/. ~/ur5e_pickplace/
cd ~/ur5e_pickplace
ls -R
```

You should see `README.md`, `config/`, `docs/`, `scripts/`.

If `$KIT` came back empty, use the download buttons on the file cards in Cowork
to save them to your Windows Downloads folder, then:

```bash
cp -r /mnt/c/Users/sachi/Downloads/<whatever you saved> ~/ur5e_pickplace/
```

> **Copy these out sooner rather than later.** That
> `local-agent-mode-sessions` directory is Cowork's scratch area and is not
> guaranteed to survive between sessions.

---

## Step 3 — make the scripts executable and check line endings

```bash
cd ~/ur5e_pickplace
chmod +x scripts/*.sh

# The kit was written with Unix line endings, but if it round-tripped through a
# Windows editor this catches it. A stray \r makes bash fail with the useless
# error "bad interpreter: /usr/bin/env bash^M".
file scripts/*.sh          # want: "ASCII text", NOT "with CRLF line terminators"
sed -i 's/\r$//' scripts/*.sh config/*.yaml     # harmless if already clean
```

---

## Step 4 — git init and hygiene

```bash
cd ~/ur5e_pickplace
git init
bash scripts/01_git_hygiene.sh     # writes .claude/settings.json, no Co-Authored-By
git add -A
git commit -m "M-1 kit: reference report, scene config, recon and M0 verification"
```

---

## Step 5 — install the stack

```bash
bash scripts/02_bootstrap_noble.sh 2>&1 | tee bootstrap_$(date +%Y%m%d_%H%M%S).log
```

Expect 20–40 minutes and several sudo prompts. Exit code 0 means verified; any
`[FAIL]` line means stop and read it.

**If colcon build gets OOM-killed** — WSL2 defaults to a memory cap that a
parallel MoveIt build can exceed. Either limit parallelism:

```bash
colcon build --symlink-install --parallel-workers 2
```

or raise the cap by creating `C:\Users\sachi\.wslconfig` on the Windows side:

```ini
[wsl2]
memory=12GB
processors=8
```

then `wsl --shutdown` from PowerShell and reopen the terminal.

---

## Step 6 — Gazebo rendering smoke test (WSL-specific)

Before touching the robot, confirm Gazebo can render at all:

```bash
source /opt/ros/jazzy/setup.bash
gz sim shapes.sdf
```

A window with shapes should appear via WSLg.

**If it fails or is unusably slow**, Gazebo Harmonic's default OGRE2 renderer is
not getting hardware acceleration. In order of preference:

```bash
# 1. Check what GL driver you actually got
sudo apt install -y mesa-utils
glxinfo -B | grep -E 'OpenGL renderer|OpenGL version'
#    "D3D12 (...)" = GPU passthrough working
#    "llvmpipe"    = pure software, will be very slow

# 2. Fall back to the older renderer, which is far more tolerant in WSL
gz sim --render-engine ogre shapes.sdf

# 3. Last resort — force software rendering (correct, but slow)
export LIBGL_ALWAYS_SOFTWARE=1
```

Make sure your **Windows** GPU driver is current (NVIDIA/AMD/Intel all ship
WSL-aware drivers). WSL gets GPU access through the Windows driver, so updating
inside Ubuntu does nothing.

### Run milestone data collection headless

This is the important part for this project. Every milestone's evidence — the
20/20 planning log, TCP pose vs. commanded, the M3 slip CSV, the M5 repeatability
run — comes from **state topics, not from pixels**. So run the sim headless for
data and only open the GUI for spot checks:

```bash
gz sim -s <world>          # server only, no GUI
```

Headless sidesteps WSL rendering entirely and runs considerably faster, which
matters when M3 and M5 each want 20 consecutive cycles. Use the GUI when you
need to *see* something, and take your screenshots from a GUI run — but note in
the log which run a screenshot came from, since the spec requires a screenshot
and its claims to be reconcilable against the same run.

---

## Step 7 — arm-only sanity check

Before adding a gripper, confirm UR's own supported path works:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur5e_ws/install/setup.bash          # only if bootstrap built from source
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur5e
```

RViz with a UR5e and a working MoveIt planning session. **If this doesn't come
up, stop here.** The problem is upstream of anything this project adds, and
every later milestone would inherit it.

---

## Step 8 — recon

```bash
cd ~/ur5e_pickplace
source /opt/ros/jazzy/setup.bash
mkdir -p docs
bash scripts/00_recon.sh 2>&1 | tee docs/recon_$(date +%Y%m%d_%H%M%S).log
```

Read-only: installs nothing, launches no simulation. Send back that log.

It answers the five open questions in `docs/M-1_reference_report.md` §6 — UR
macro signature, robotiq package source, flange frame and mount transform, the
real actuated gripper joint's limits, and the five mimic multipliers. The URDF
merge gets written from those observed values, not from the donor repo's README.

---

## Troubleshooting

**Wrong Ubuntu version.** WSL's default `Ubuntu` distro may be 22.04. Install
Noble explicitly from PowerShell:

```powershell
wsl --install -d Ubuntu-24.04
wsl -l -v                  # confirm VERSION 2, not 1
```

WSL **1** will not work — it has no real kernel and Gazebo's physics and
networking assumptions break. `wsl --set-version Ubuntu-24.04 2` if needed.

**`ros2` not found in a new terminal.** The bootstrap appends the setup line to
`~/.bashrc`, which only applies to terminals opened afterwards. Either open a new
one or `source ~/.bashrc`.

**Clock skew after Windows sleep.** WSL's clock can drift when the host
suspends, which makes apt reject repository signatures and can upset ROS
timestamps. Fix:

```bash
sudo hwclock -s
```

**Claude Code can't see the folder.** Start it from inside the project:

```bash
cd ~/ur5e_pickplace && claude
```
