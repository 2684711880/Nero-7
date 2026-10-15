# Source from bash or zsh: source /path/to/Nero-7/scripts/official_control_env.sh
# BASH_SOURCE is unavailable in zsh; use its source-file expansion there.
if [ -n "${ZSH_VERSION:-}" ]; then
  nero_script_path="${(%):-%x}"
else
  nero_script_path="${BASH_SOURCE[0]}"
fi
nero_project_root="$(cd -- "$(dirname -- "$nero_script_path")/.." && pwd)"
if [ ! -f "$nero_project_root/hardware_ws/install/setup.sh" ] ||
   [ ! -d "$nero_project_root/.venv-official" ]; then
  echo 'Run bash scripts/build_official_control.sh first.' >&2
  return 1
fi
if [ -n "${ZSH_VERSION:-}" ]; then
  source /opt/ros/humble/setup.zsh
  source "$nero_project_root/hardware_ws/install/setup.zsh"
else
  source /opt/ros/humble/setup.bash
  source "$nero_project_root/hardware_ws/install/setup.bash"
fi
nero_python_site="$("$nero_project_root/.venv-official/bin/python" -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
export PYTHONPATH="$nero_project_root/third_party/pyAgxArm:$nero_python_site${PYTHONPATH:+:$PYTHONPATH}"
unset nero_script_path nero_project_root nero_python_site
