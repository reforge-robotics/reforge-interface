# MOTOMAN NEXT NEX7 model provenance

- Model identifier: `NEX07C00`
- Supplied archive: `yaskawa/URDF/NEX07C00/nex07c00.zip`
- Archive SHA-256:
  `e66da5f19c31ece8906a98f88cecb952a94956fc1a8abc745946e49932ce95ce`
- Original vendor URDF SHA-256:
  `5b6dfeb0a876ba250025db1ffe5487e0d9e2b88c169ce38962ff90c94159841b`
- Reforge-derived URDF SHA-256:
  `972cb8776c68744c8864d9d22fae05d627cd4b2d82b861ba05e8b3618951d60a`
- Derived model edits: renamed the six moving child links to `S_link`,
  `L_link`, `U_link`, `R_link`, `B_link`, and `T_link` while retaining
  controller joint names; added a fixed, coincident `tool0` child of
  `flange`. Link geometry, joint origins, axes, and limits were not changed.
- The 28 referenced visual/collision OBJ and MTL files remain the supplied
  files without edits.
- Source: Yaskawa materials supplied to Reforge; local archive inspected
  2026-09-28.
- Public redistribution approval: the user reported that Yaskawa explicitly
  approved use of these files in the public `reforge-interface:yaskawa-sdk`
  branch. Source: User, 2026-09-28.

The assets remain Yaskawa-supplied material and are not relicensed by the
repository license. Model availability is not motion approval: confirm the
installed variant, controller-discovered joint contract, tool/payload,
workspace, and trajectory before commanding hardware.
