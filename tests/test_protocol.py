from zelux_drivers.radxa.servo import checksum, frame, sync_write_frame


def test_checksum():
    assert checksum(bytes([1, 2, 3])) == 249


def test_frame_header():
    packet = frame(1, 3, b"")
    assert packet[:3] == bytes([0xFF, 0xFF, 1])


def test_sync_write_has_broadcast_id():
    packet = sync_write_frame(42, 2, [(1, b"\x00\x08"), (2, b"\x10\x08")])
    assert packet[2] == 0xFE
