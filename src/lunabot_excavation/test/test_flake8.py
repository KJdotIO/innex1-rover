from ament_flake8.main import main_with_errors


def test_flake8():
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, "\n".join(errors)
