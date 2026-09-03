from robot.run import _build_parser


def test_connect_test_accepts_explicit_robot_id() -> None:
    """Verify the customer supplies its robot identifier explicitly.

    Args:
        None.

    Returns:
        `None`.

    Raises:
        AssertionError: If `connect_test` discards the supplied identifier.
    """
    parser = _build_parser()

    args = parser.parse_args(
        [
            "connect_test",
            "robot-host:3000",
            "--sdk_token",
            "token",
            "--robot_id",
            "customer_bot",
        ]
    )

    assert args.robot_id == "customer_bot"
