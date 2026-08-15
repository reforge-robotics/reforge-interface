from robot.robot_interface import BOT_ID
from robot.run import _build_parser


def test_connect_test_defaults_to_configured_robot_id() -> None:
    """Verify omitted `--robot_id` keeps the configured robot identifier.

    Args:
        None.

    Returns:
        `None`.

    Raises:
        AssertionError: If `connect_test` overrides `BOT_ID` with an empty
            default.
    """
    parser = _build_parser()

    args = parser.parse_args(
        [
            "connect_test",
            "10.0.0.4:3000",
            "--sdk_token",
            "token",
        ]
    )

    assert args.robot_id == BOT_ID
