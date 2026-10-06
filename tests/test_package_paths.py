"""Unit tests for core.package_paths — locating the source checkout without a hard-coded path."""
from skuba_navigation.core.package_paths import ENV_VAR, find_package_source_dir

PKG_XML = "<package><name>skuba_navigation</name></package>\n"


def _make_ws(tmp_path, source_rel="src/skuba-newgen/skuba-navigation"):
    source = tmp_path / source_rel
    source.mkdir(parents=True)
    (source / "package.xml").write_text(PKG_XML)
    share = tmp_path / "install/skuba_navigation/share/skuba_navigation"
    share.mkdir(parents=True)
    return source, share


def test_finds_nested_renamed_checkout_by_package_xml_name(tmp_path):
    source, share = _make_ws(tmp_path)
    (share / "package.xml").write_text(PKG_XML)  # plain copy, not a symlink
    assert find_package_source_dir(share, "skuba_navigation", env={}) == source.resolve()


def test_follows_symlink_install_package_xml(tmp_path):
    source, share = _make_ws(tmp_path, "elsewhere/skuba-navigation")
    (share / "package.xml").symlink_to(source / "package.xml")
    assert find_package_source_dir(share, "skuba_navigation", env={}) == source.resolve()


def test_env_var_overrides_discovery(tmp_path):
    _source, share = _make_ws(tmp_path)
    override = tmp_path / "custom"
    override.mkdir()
    assert find_package_source_dir(share, "skuba_navigation", env={ENV_VAR: str(override)}) == override.resolve()


def test_skips_colcon_ignored_and_build_dirs(tmp_path):
    share = tmp_path / "install/skuba_navigation/share/skuba_navigation"
    share.mkdir(parents=True)
    ignored = tmp_path / "src/old_copy"
    ignored.mkdir(parents=True)
    (ignored / "package.xml").write_text(PKG_XML)
    (ignored / "COLCON_IGNORE").write_text("")
    assert find_package_source_dir(share, "skuba_navigation", env={}) is None


def test_returns_none_when_no_checkout_declares_the_package(tmp_path):
    share = tmp_path / "install/skuba_navigation/share/skuba_navigation"
    share.mkdir(parents=True)
    other = tmp_path / "src/other_pkg"
    other.mkdir(parents=True)
    (other / "package.xml").write_text("<package><name>other_pkg</name></package>")
    assert find_package_source_dir(share, "skuba_navigation", env={}) is None
