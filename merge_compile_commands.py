import json, glob, os, sys

roots = [
  # "/home/zach-ubuntu/ros2_ws/build/wpr_simulation2/compile_commands.json",
  # "/home/zach-ubuntu/ros2_ws/build/my_pkg/compile_commands.json",
  "/home/zach-ubuntu/ros_op_system_ws/**/compile_commands.json",
]

files = []
for pat in roots:
  files += glob.glob(pat, recursive=True)

# De-dup file list while keeping order
seen = set()
uniq = []
for f in files:
  f = os.path.normpath(f)
  if f not in seen:
    uniq.append(f); seen.add(f)

out = []
seen_entries = set()
used = 0

for path in uniq:
  try:
    with open(path, "r") as fp:
      data = json.load(fp)
  except Exception as e:
    # Keep going, but warn
    print(f"[warn] skip {path}: {e}", file=sys.stderr)
    continue

  if not isinstance(data, list):
    print(f"[warn] skip {path}: not a list", file=sys.stderr)
    continue

  used += 1
  for e in data:
    cmd = e.get("command")
    if cmd is None:
      args = e.get("arguments", [])
      cmd = " ".join(args) if isinstance(args, list) else str(args)
    key = (e.get("directory"), e.get("file"), cmd)
    if key in seen_entries:
      continue
    seen_entries.add(key)
    out.append(e)

dst = os.path.abspath("./compile_commands.merged.json")
with open(dst, "w") as f:
  json.dump(out, f, indent=2)

print("Merged DB written to:", dst)
print("Source files found:", len(uniq))
print("Source DBs used:", used)
print("Entries:", len(out))
