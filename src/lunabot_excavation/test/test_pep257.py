from ament_pep257.main import main


def test_pep257():
    rc = main(argv=[".", "test"])
    assert rc == 0
