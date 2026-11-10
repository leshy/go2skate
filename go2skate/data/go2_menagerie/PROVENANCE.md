# go2_menagerie — provenance

The Unitree Go2 model from Google DeepMind's MuJoCo Menagerie, vendored so
that the measured plant in `dimos/robot/unitree/go2/sim` is reproducible: its
fitted knobs are DELTAS on this exact scene, and every number in that
package's README was produced against these bytes.

- upstream:  https://github.com/google-deepmind/mujoco_menagerie
- commit:    4c358ef9d9d7f32ca58b40b490884a0c1726a440 ("internal (#288)", 2026-06-04)
- vendored:  2026-08-17
- contents:  the `unitree_go2/` subtree and the repository `LICENSE`,
             byte-identical to the git tree at that commit (verified per file
             by blob hash at vendoring time, and pinned by the tree hash below)

## License

`LICENSE` in this directory is the menagerie repository's aggregate license
file, copied unmodified: one section per model directory. The section for
`unitree_go2/` is **BSD-3-Clause, Copyright (c) 2016-2022 HangZhou YuShu
TECHNOLOGY CO.,LTD. ("Unitree Robotics")** — redistribution permitted with
the copyright notice and disclaimer retained, which shipping this file
verbatim satisfies. (Not Apache 2.0: that is the license of menagerie's own
tooling, not of the Unitree-contributed model.)

## Tree hash

sha256 of the sorted per-file manifest (`sha256  relpath` lines, LF-joined,
every file except this one):

    ea91fde0e64a51b9d6a5df52696a0cffe519a6f9e6223cd341b7a82d630074ca

(Trimmed for go2skate, 2026-11-10: go2.png, go2_mjx.png, go2_mjx.xml and scene_mjx.xml
removed, unused here; every remaining file is still byte-identical to upstream. The
full subtree's hash was 4ef7ea987b7636c53b7c93910eacdf6e5c0cb763e9f3a7b241674b2c2f1d4a9b.)

Recompute with:

    cd data/go2_menagerie
    find . -type f -not -name PROVENANCE.md | sed 's|^\./||' | sort \
      | while read -r f; do printf '%s  %s\n' "$(sha256sum "$f" | cut -d' ' -f1)" "$f"; done \
      | sha256sum

`dimos/robot/unitree/go2/sim/test_vendor.py` asserts this hash against
`MENAGERIE_TREE_SHA256` in `model.py`; if either changes, every fitted number
in the plant is suspect and must be re-baselined deliberately, never silently.
