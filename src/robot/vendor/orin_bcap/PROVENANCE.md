# DENSO ORiN b-CAP Python client provenance

This directory vendors the required Python b-CAP TCP client source from
[DENSORobot/orin_bcap](https://github.com/DENSORobot/orin_bcap) at commit
`18c503c37d41ed0dc2a9d1f9ea97d30186042ae0` (2020-12-24).

Vendored files:

- `bcapclient.py`
- `orinexception.py`
- `variant.py`

The source is licensed under the MIT License; the original license is retained
in each source file and reproduced in `LICENSE`. The local modifications change
the two internal imports in `bcapclient.py` to package-relative imports, trim
three trailing-whitespace lines, normalize the final newline in `variant.py`,
and add an explicit `close()` method so the wrapper can deterministically
release the TCP socket. The original destructor delegates to that method.

The Cobotta integration owns CRC9/RC9-specific commands, lifecycle policy, and
all robot behavior. This vendored code is only the generic b-CAP packet and TCP
client implementation.
