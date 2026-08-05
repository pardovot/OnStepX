# Writes flash_images.txt next to firmware.bin listing every image the upload
# would write, as "<offset> <path>" lines, and flash_params.txt with the mode,
# frequency and size. The release workflow feeds both to esptool merge-bin, so
# the merged image always matches this build rather than values duplicated in
# the workflow that can drift out of step with platformio.ini.

Import("env")
import os


def dump_flash_images(source, target, env):
    build_dir = env.subst("$BUILD_DIR")
    rows = []

    # forward slashes so the paths stay usable from shell scripts on Windows too
    def norm(p):
        return os.path.abspath(p).replace("\\", "/")

    for offset, image in env.get("FLASH_EXTRA_IMAGES", []):
        rows.append((env.subst(offset), norm(env.subst(image))))

    app_offset = env.subst("$ESP32_APP_OFFSET") or "0x10000"
    rows.append((app_offset, norm(os.path.join(build_dir, "firmware.bin"))))

    rows.sort(key=lambda r: int(r[0], 16))

    # newline="\n" so the file is LF even on Windows, it is parsed by shell
    with open(os.path.join(build_dir, "flash_images.txt"), "w", newline="\n") as f:
        for offset, path in rows:
            f.write("%s %s\n" % (offset, path))

    board = env.BoardConfig()
    mode = env.subst("$BOARD_FLASH_MODE") or board.get("build.flash_mode", "dio")
    size = board.get("upload.flash_size", "4MB")

    # BOARD_F_FLASH comes through as "80000000L", esptool wants "80m"
    freq = env.subst("$BOARD_F_FLASH") or str(board.get("build.f_flash", "40000000L"))
    digits = "".join(c for c in freq if c.isdigit())
    freq = "%dm" % (int(digits) // 1000000) if digits else "40m"

    with open(os.path.join(build_dir, "flash_params.txt"), "w", newline="\n") as f:
        f.write("mode %s\nfreq %s\nsize %s\n" % (mode, freq, size))


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", dump_flash_images)
