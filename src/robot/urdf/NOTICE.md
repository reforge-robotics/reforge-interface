# Yaskawa customer-review candidate notice

The Python protobuf and gRPC files in this candidate are generated from the
Reforge-owned `reforge.yaskawa.bridge.v1` contract. They contain no Yaskawa SDK
protobuf definition or generated vendor binding.

The bundled `NEX07C00` URDF, visual meshes, collision meshes, and material files
were supplied for the MOTOMAN NEXT NEX7. Yaskawa's explicit approval to use
these assets in this public branch was reported by the user on 2026-09-28. The
assets remain Yaskawa-supplied material and are not relicensed by the repository
license. Exact source hashes and provenance are in
`NEX07C00/PROVENANCE.md`.

Availability of the supplied model does not establish that the connected robot
is the matching variant or validate joint ordering, signs, limits, dynamics,
payload/tool configuration, workspace, or trajectory. Those checks remain
mandatory before live motion.

Production publication and hardware motion for this target remain disabled.
