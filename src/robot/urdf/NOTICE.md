# Yaskawa customer-review candidate notice

The Python protobuf and gRPC files in this candidate are generated from the
Reforge-owned `reforge.yaskawa.bridge.v1` contract. They contain no Yaskawa SDK
protobuf definition or generated vendor binding.

The bundled `test_robot.urdf` is the generic Reforge interface fixture. It is
included only so installation and hardware-free package checks can load the
adapter. It is not a Yaskawa NEX7 model and must not be used to approve a
calibration trajectory or live motion. The actual NEX7 description and meshes
remain excluded until the controller variant and redistribution permission are
confirmed.

Production publication and hardware motion for this target remain disabled.
