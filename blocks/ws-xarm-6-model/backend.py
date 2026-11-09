# ── EsaSim block ──────────────────────────────────────────────────────
# xArm model (ws-xarm-6-model) · version 2 · machine block, Python: def run(ctx)
# The UFACTORY xArm 6 or xArm 7 for the digital twin: kinematics and 3D meshes (UFACTORY's own, BSD-3), for the 3D view and the simulated camera.
# Workspace: Gianluca De Novi's workspace · Author: Gianluca De Novi
# Saved: 2026-11-09
#
# Inputs   ctx.inputs["id"] is a list of files: .path .name .size .read_text() .read_json()
#   (none)
# Outputs  ctx.output("id", "name.ext") is the path to write; ctx.write_json / ctx.write_text
#   model       Files (gz, txt)           Robot model · xarm6.robot.json.gz or xarm7.robot.json.gz (esasim.robot/1) and LICENSE-xarm_ros.txt
#   telemetry   Live telemetry            ctx.telemetry("channel", data) while it runs
# Settings ctx.params["key"]
#   arm         select    Arm = "xArm 6" of xArm 6 | xArm 7
#   detail      number    Detail (mm) = 3
# Runs on  Any machine. Return a dict to fill the "result" output.
# Packages numpy (installed by the worker with pip when missing)
# EsaSim rewrites these lines from the block's definition each time it is saved: change the definition, not them.
# ── end of EsaSim block header ────────────────────────────────────────

"""xArm model: a UFACTORY xArm 6 or xArm 7 for the digital twin and its 3D view. Downloads UFACTORY's meshes of the
arm (xarm_ros, BSD-3-Clause) once into the worker's data folder, merges their small details (the 3D view and the
simulated camera do not need them), and writes one small file with the arm's kinematics and meshes."""
import gzip
import json
import os


def cache_folder(arm):
    here = os.path.abspath(os.getcwd())                    # <data>/jobs/<job>/<task>
    jobs = os.path.dirname(os.path.dirname(here))
    base = os.path.dirname(jobs) if os.path.basename(jobs) == "jobs" else os.path.dirname(here)
    return os.path.join(base, "cache", arm)


def run(ctx):
    p = ctx.params
    arm = "xarm7" if "7" in str(p.get("arm", "xArm 6")) else "xarm6"
    cell = max(1.0, min(20.0, float(p.get("detail", 3))))
    try:
        model = build_model(cache_folder(arm), cell, ctx.log, arm)
    except Exception as e:
        raise RuntimeError("Could not get the %s meshes (%s). The worker downloads them once from GitHub "
                           "(raw.githubusercontent.com): check that this machine reaches it." % (ARMS[arm]["name"], e))
    raw = json.dumps(model, separators=(",", ":")).encode()
    with open(ctx.output("model", "%s.robot.json.gz" % arm), "wb") as f:
        f.write(gzip.compress(raw, 9))
    ctx.write_text("model", "LICENSE-xarm_ros.txt", XARM_LICENSE + "\n")
    tris = sum(l["triangles"] for l in model["links"])
    ctx.log("%s model: %d joints, %d triangles (from %d), %d KB." % (model["robot"], len(model["joints"]), tris,
                                                                   sum(l["original"] for l in model["links"]), len(gzip.compress(raw, 9)) // 1024))
    return {"robot": model["robot"], "joints": len(model["joints"]), "triangles": tris, "detail_mm": cell}


# Nominal kinematics of xarm_ros (xarm_description/urdf/xarmN/xarmN.urdf.xacro): each joint's origin (mm) and rpy,
# the joint turning about its own z; limits in degrees.
ARMS = {
    "xarm6": {"name": "UFACTORY xArm 6",
              "joints": [((0.0, 0.0, 267.0), (0.0, 0.0, 0.0)), ((0.0, 0.0, 0.0), (-1.5708, 0.0, 0.0)), ((53.5, -284.5, 0.0), (0.0, 0.0, 0.0)),
                         ((77.5, 342.5, 0.0), (-1.5708, 0.0, 0.0)), ((0.0, 0.0, 0.0), (1.5708, 0.0, 0.0)), ((76.0, 97.0, 0.0), (-1.5708, 0.0, 0.0))],
              "limits": [(-178.2, 178.2), (-118.0, 120.0), (-178.2, 11.0), (-178.2, 178.2), (-97.0, 178.2), (-178.2, 178.2)]},
    "xarm7": {"name": "UFACTORY xArm 7",
              "joints": [((0.0, 0.0, 267.0), (0.0, 0.0, 0.0)), ((0.0, 0.0, 0.0), (-1.5708, 0.0, 0.0)), ((0.0, -293.0, 0.0), (1.5708, 0.0, 0.0)),
                         ((52.5, 0.0, 0.0), (1.5708, 0.0, 0.0)), ((77.5, -342.5, 0.0), (1.5708, 0.0, 0.0)), ((0.0, 0.0, 0.0), (1.5708, 0.0, 0.0)),
                         ((76.0, 97.0, 0.0), (-1.5708, 0.0, 0.0))],
              "limits": [(-360.0, 360.0), (-118.0, 120.0), (-360.0, 360.0), (-11.0, 225.0), (-360.0, 360.0), (-97.0, 180.0), (-360.0, 360.0)]},
}

MESH_URL = "https://raw.githubusercontent.com/xArm-Developer/xarm_ros/master/xarm_description/meshes/%s/visual/%s.stl"

XARM_LICENSE = ("xArm meshes and kinematics: xarm_ros by UFACTORY (https://github.com/xArm-Developer/xarm_ros), "
                "Copyright (c) 2018, UFACTORY Inc. All rights reserved. BSD 3-Clause License: redistribution and use in "
                "source and binary forms, with or without modification, are permitted provided that the copyright notice, "
                "the conditions and the disclaimer are retained; the name of the copyright holder may not be used to "
                "endorse products without permission. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "
                "\"AS IS\" AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED.")

# kept for the blocks that import these names from this one
XARM6_JOINTS, XARM6_LIMITS = ARMS["xarm6"]["joints"], ARMS["xarm6"]["limits"]
XARM6_LINKS = ["link_base", "link1", "link2", "link3", "link4", "link5", "link6"]
XARM6_MESH_URL = MESH_URL.replace("/meshes/%s/", "/meshes/xarm6/")
XARM6_LICENSE = XARM_LICENSE


def read_stl(data):
    """Triangles (N x 3 x 3, float32) of a binary or text STL."""
    import numpy as np
    if data[:5].lower() == b"solid" and b"facet" in data[:400]:
        nums = [float(x) for line in data.decode("ascii", "replace").splitlines() if line.strip().startswith("vertex") for x in line.split()[1:4]]
        return np.array(nums, np.float32).reshape(-1, 3, 3)
    n = int(np.frombuffer(data[80:84], "<u4")[0])
    rec = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    return rec["v"].copy()


def decimate(tris, cell):
    """Vertex clustering: vertices closer than `cell` merge, triangles that collapse go. Returns (vertices, faces)."""
    import numpy as np
    v = tris.reshape(-1, 3).astype(np.float64)
    keys = np.floor(v / cell).astype(np.int64)
    uniq, inv = np.unique(keys, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    sums = np.zeros((len(uniq), 3)); np.add.at(sums, inv, v)
    cnt = np.bincount(inv, minlength=len(uniq))[:, None]
    verts = sums / cnt
    f = inv.reshape(-1, 3)
    f = f[(f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])]
    if len(f):
        _, idx = np.unique(np.sort(f, axis=1), axis=0, return_index=True)
        f = f[np.sort(idx)]
    used = np.unique(f)
    remap = -np.ones(len(verts), np.int64); remap[used] = np.arange(len(used))
    return verts[used], remap[f]


def build_model(cache, cell=2.5, log=print, arm="xarm6"):
    """The arm's model (esasim.robot/1): kinematics and decimated meshes in mm. Downloads UFACTORY's meshes once
    into `cache`."""
    import urllib.request
    a = ARMS[arm]
    os.makedirs(cache, exist_ok=True)
    links = []
    for name in ["link_base"] + ["link%d" % (i + 1) for i in range(len(a["joints"]))]:
        path = os.path.join(cache, name + ".stl")
        if not os.path.exists(path) or os.path.getsize(path) < 1000:
            url = MESH_URL % (arm, name)
            log("Downloading %s" % url)
            with urllib.request.urlopen(url, timeout=60) as r:
                data = r.read()
            with open(path + ".part", "wb") as f:
                f.write(data)
            os.replace(path + ".part", path)
        tris = read_stl(open(path, "rb").read()) * 1000.0  # metres -> mm
        v, f = decimate(tris, cell)
        links.append({"name": name, "v": [int(round(x * 10)) for x in v.reshape(-1)], "f": [int(x) for x in f.reshape(-1)], "triangles": int(len(f)), "original": int(len(tris))})
    return {"kind": "esasim.robot/1", "robot": a["name"], "units": "mm", "scale": 0.1,
            "joints": [{"xyz": list(xyz), "rpy": list(rpy), "axis": [0, 0, 1], "limits": list(lim)} for (xyz, rpy), lim in zip(a["joints"], a["limits"])],
            "links": links, "license": XARM_LICENSE}
